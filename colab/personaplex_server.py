"""
CASE Personaplex Bridge Server
Bridges ESP32 raw PCM audio <-> Moshi/Personaplex Opus WebSocket

Protocol (from Moshi server.py source):
  - WebSocket at /api/chat?text_prompt=...&voice_prompt=...
  - Binary framing: first byte = kind tag
      0x00  Server->Client  Handshake (server ready)
      0x01  Both            Opus audio frame
      0x02  Server->Client  Text token (UTF-8)
  - Audio: Opus via sphn at 24 kHz mono

Setup on Colab:
  1. Moshi running:   python -m moshi.server --port 8998
  2. Paste this file into a Colab cell
  3. Run:             await start_server()
  4. In another cell: !cloudflared tunnel --url http://127.0.0.1:9001
  5. Copy tunnel URL to ESP32 COLAB_SERVER_URL
"""

import asyncio
import logging
import time
import numpy as np
from urllib.parse import quote

from aiohttp import web
import aiohttp

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
log = logging.getLogger("bridge")

# ── Configuration ────────────────────────────────────────────
MOSHI_HOST = "localhost"
MOSHI_PORT = 8998
PROXY_PORT = 9001

ESP32_SAMPLE_RATE = 16000
MOSHI_SAMPLE_RATE = 24000

SYSTEM_PROMPT = (
    "You are CASE, a self-balancing robot from Interstellar. "
    "You are helpful, witty, and occasionally sarcastic. "
    "Humor setting: 75%. Keep responses concise (under 20 seconds). "
    "Be conversational and show personality."
)

# Just the filename — Moshi resolves the full path from --voice-prompt-dir
VOICE_PROMPT = "NATF0.pt"

# Binary message kind tags (from Moshi server.py)
KIND_HANDSHAKE = 0x00
KIND_AUDIO     = 0x01
KIND_TEXT      = 0x02


# ── Audio resampling ─────────────────────────────────────────
def resample(audio, src_rate, dst_rate):
    if src_rate == dst_rate:
        return audio
    ratio = dst_rate / src_rate
    n_out = int(len(audio) * ratio)
    idx = np.clip((np.arange(n_out) / ratio).astype(int), 0, len(audio) - 1)
    return audio[idx]


# ── Moshi message handling ───────────────────────────────────
def _handle_message(msg, opus_reader, chunks):
    if msg.type == aiohttp.WSMsgType.BINARY and len(msg.data) > 0:
        kind = msg.data[0]
        payload = msg.data[1:]
        if kind == KIND_AUDIO and len(payload) > 0:
            opus_reader.append_bytes(payload)
            pcm = opus_reader.read_pcm()
            if pcm is not None and len(pcm) > 0:
                chunks.append(pcm)
        elif kind == KIND_TEXT:
            log.info(f"  Moshi says: {payload.decode('utf-8', errors='replace')}")
    elif msg.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
                      aiohttp.WSMsgType.ERROR):
        log.info(f"Moshi connection ended: {msg.type}")


async def _drain_responses(ws, opus_reader, chunks):
    while True:
        try:
            msg = await asyncio.wait_for(ws.receive(), timeout=0.001)
            _handle_message(msg, opus_reader, chunks)
        except asyncio.TimeoutError:
            break


# ── Core: send audio to Moshi, get response ──────────────────
async def process_with_moshi(pcm_int16_16k):
    import sphn

    # Resample 16 kHz -> 24 kHz, convert to float32 [-1, 1]
    pcm_24k = resample(pcm_int16_16k, ESP32_SAMPLE_RATE, MOSHI_SAMPLE_RATE)
    pcm_float = pcm_24k.astype(np.float32) / 32768.0

    # WebSocket URL with prompts as query parameters
    ws_url = (
        f"ws://{MOSHI_HOST}:{MOSHI_PORT}/api/chat"
        f"?text_prompt={quote(SYSTEM_PROMPT)}"
        f"&voice_prompt={quote(VOICE_PROMPT)}"
    )
    log.info("Connecting to Moshi...")

    opus_writer = sphn.OpusStreamWriter(MOSHI_SAMPLE_RATE)
    opus_reader = sphn.OpusStreamReader(MOSHI_SAMPLE_RATE)
    response_chunks = []

    try:
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(ws_url, max_msg_size=10 * 1024 * 1024) as ws:
                log.info("Connected, waiting for handshake (0x00)...")

                # Wait for handshake byte
                msg = await asyncio.wait_for(ws.receive(), timeout=30.0)
                if msg.type != aiohttp.WSMsgType.BINARY or msg.data[0] != KIND_HANDSHAKE:
                    log.error(f"Bad handshake: type={msg.type}, "
                              f"data={msg.data[:20] if msg.data else None}")
                    return None
                log.info("Handshake OK — streaming audio to Moshi")

                # Feed audio in 20ms chunks with real-time pacing
                chunk_samples = int(MOSHI_SAMPLE_RATE * 0.02)  # 480 samples
                total_chunks = 0

                for i in range(0, len(pcm_float), chunk_samples):
                    chunk = pcm_float[i : i + chunk_samples]
                    if len(chunk) < chunk_samples:
                        chunk = np.pad(chunk, (0, chunk_samples - len(chunk)))

                    opus_writer.append_pcm(chunk)
                    opus_bytes = opus_writer.read_bytes()
                    if opus_bytes and len(opus_bytes) > 0:
                        await ws.send_bytes(bytes([KIND_AUDIO]) + opus_bytes)
                        total_chunks += 1

                    await asyncio.sleep(0.02)
                    await _drain_responses(ws, opus_reader, response_chunks)

                log.info(f"Sent {total_chunks} chunks, sending trailing silence...")

                # Send 1s of silence so Moshi knows we stopped talking
                silence = np.zeros(chunk_samples, dtype=np.float32)
                for _ in range(50):
                    opus_writer.append_pcm(silence)
                    opus_bytes = opus_writer.read_bytes()
                    if opus_bytes and len(opus_bytes) > 0:
                        await ws.send_bytes(bytes([KIND_AUDIO]) + opus_bytes)
                    await asyncio.sleep(0.02)
                    await _drain_responses(ws, opus_reader, response_chunks)

                # Collect remaining response (up to 15s)
                log.info("Collecting Moshi response...")
                deadline = time.time() + 15
                while time.time() < deadline:
                    try:
                        msg = await asyncio.wait_for(ws.receive(), timeout=3.0)
                        _handle_message(msg, opus_reader, response_chunks)
                    except asyncio.TimeoutError:
                        log.info("No more audio (3s silence)")
                        break

    except asyncio.TimeoutError:
        log.error("Moshi handshake timeout (30s)")
        return None
    except Exception as e:
        log.error(f"Moshi error: {e}", exc_info=True)
        return None

    if not response_chunks:
        log.warning("No audio response from Moshi")
        return None

    # Combine, resample 24 kHz -> 16 kHz, convert to int16
    full = np.concatenate(response_chunks)
    resampled = resample(full, MOSHI_SAMPLE_RATE, ESP32_SAMPLE_RATE)
    result = np.clip(resampled * 32767, -32768, 32767).astype(np.int16)
    log.info(f"Response: {len(result)} samples ({len(result)/ESP32_SAMPLE_RATE:.1f}s)")
    return result


# ── HTTP handlers ────────────────────────────────────────────
async def handle_health(request):
    return web.Response(text="OK")


async def handle_audio(request):
    body = await request.read()
    if len(body) < 2:
        return web.Response(status=400, text="No audio data")

    pcm = np.frombuffer(body, dtype=np.int16)
    log.info(f"ESP32: {len(pcm)} samples ({len(pcm)/ESP32_SAMPLE_RATE:.1f}s)")

    response = await process_with_moshi(pcm)

    if response is not None and len(response) > 0:
        return web.Response(
            body=response.tobytes(),
            content_type="application/octet-stream",
        )
    return web.Response(status=204)


# ── Server startup ───────────────────────────────────────────
async def start_server():
    """Async entry — works in both Colab cells (await) and terminal."""
    app = web.Application(client_max_size=10 * 1024 * 1024)
    app.router.add_get("/health", handle_health)
    app.router.add_post("/api/audio", handle_audio)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", PROXY_PORT)
    await site.start()

    log.info("=" * 55)
    log.info("  CASE Personaplex Bridge — RUNNING")
    log.info("=" * 55)
    log.info(f"  Moshi:  ws://{MOSHI_HOST}:{MOSHI_PORT}/api/chat")
    log.info(f"  Proxy:  http://0.0.0.0:{PROXY_PORT}")
    log.info(f"  Voice:  {VOICE_PROMPT}")
    log.info("=" * 55)

    try:
        await asyncio.Event().wait()
    finally:
        await runner.cleanup()


# ── Entry point ──────────────────────────────────────────────
# Works in both Colab cells and terminal — just paste & run.
# In Colab, nest_asyncio patches the event loop so run_app() works.
def main():
    try:
        import nest_asyncio
        nest_asyncio.apply()
    except ImportError:
        pass  # Not needed in terminal

    app = web.Application(client_max_size=10 * 1024 * 1024)
    app.router.add_get("/health", handle_health)
    app.router.add_post("/api/audio", handle_audio)

    log.info("=" * 55)
    log.info("  CASE Personaplex Bridge — STARTING")
    log.info("=" * 55)
    log.info(f"  Moshi:  ws://{MOSHI_HOST}:{MOSHI_PORT}/api/chat")
    log.info(f"  Proxy:  http://0.0.0.0:{PROXY_PORT}")
    log.info(f"  Voice:  {VOICE_PROMPT}")
    log.info("=" * 55)

    web.run_app(app, port=PROXY_PORT)

main()
