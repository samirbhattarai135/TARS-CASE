"""
CASE Personaplex WebSocket Server with Cloudflare Tunnel

Audio-to-audio AI conversation using Nvidia Personaplex (Moshi)
Exposed via Cloudflare Tunnel (better than ngrok for long sessions)

Setup Instructions:
1. Run this in Google Colab with GPU runtime
2. Install dependencies
3. Start Cloudflare tunnel (automatically gets public URL)
4. Copy URL to ESP32 code
"""

import asyncio
import websockets
import numpy as np
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PersonaplexServer:
    def __init__(self, voice_model="NATF2.pt"):
        """
        Initialize Personaplex server

        Args:
            voice_model: Voice model file (e.g., "NATF2.pt")
        """
        self.voice_model = voice_model
        self.sample_rate = 16000
        self.initialized = False

        logger.info("Initializing Personaplex server...")

    def initialize_personaplex(self):
        """Load Personaplex model"""
        try:
            # TODO: Load actual Personaplex model in Phase 5
            self.voice_model = PersonaplexServer.get_case_persona() 
            self.model = voice_model if self.voice_model else None

            logger.info("Personaplex model loaded successfully")
            self.initialized = True
            return True

        except Exception as e:
            logger.error(f"Failed to load Personaplex: {e}")
            return False

    def get_case_persona(self):
        """Get CASE persona/system prompt"""
        return """
        You are CASE, a self-balancing robot from Interstellar.
        You are helpful, witty, and occasionally sarcastic.
        Humor setting: 75%. Keep responses concise (under 20 seconds).
        You balance on two wheels using PID control with an MPU6050 IMU.
        You can move forward, backward, turn left and right via voice commands.
        You process simple motor commands locally, but stream complex queries to me.
        Be conversational and show personality.
        """

    async def process_audio(self, audio_bytes):
        """
        Process incoming audio through Personaplex

        Args:
            audio_bytes: Raw 16-bit PCM audio at 16kHz

        Returns:
            Response audio as bytes
        """
        try:
            # Decode audio from bytes
            audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
            logger.info(f"Received {len(audio_array)} audio samples")

            # TODO: Process through Personaplex in Phase 5
            # response_audio = self.model.process(audio_array)

            # PLACEHOLDER: Echo back a test tone
            response_audio = self.generate_test_response()

            # Encode response as bytes
            response_bytes = response_audio.astype(np.int16).tobytes()

            logger.info(f"Sending {len(response_audio)} audio samples")
            return response_bytes

        except Exception as e:
            logger.error(f"Audio processing error: {e}")
            return self.generate_error_tone()

    def generate_test_response(self):
        """Generate test audio response (placeholder)"""
        # 1 second of 440Hz tone
        duration = 1.0
        frequency = 440
        t = np.linspace(0, duration, int(self.sample_rate * duration))
        audio = (np.sin(2 * np.pi * frequency * t) * 8000).astype(np.int16)
        return audio

    def generate_error_tone(self):
        """Generate error tone"""
        duration = 0.3
        frequency = 200
        t = np.linspace(0, duration, int(self.sample_rate * duration))
        audio = (np.sin(2 * np.pi * frequency * t) * 8000).astype(np.int16)
        return audio.tobytes()


async def audio_handler(websocket, path, server):
    """Handle WebSocket connections from ESP32"""
    client_ip = websocket.remote_address[0]
    logger.info(f"New connection from {client_ip}")

    try:
        async for message in websocket:
            logger.info(f"Received message: {len(message)} bytes")

            # Process audio through Personaplex
            response = await server.process_audio(message)

            # Send response back to ESP32
            await websocket.send(response)
            logger.info("Response sent")

    except websockets.exceptions.ConnectionClosed:
        logger.info(f"Connection closed: {client_ip}")
    except Exception as e:
        logger.error(f"Error handling connection: {e}")


async def main():
    """Start WebSocket server"""
    server = PersonaplexServer()

    if not server.initialize_personaplex():
        logger.error("Failed to initialize Personaplex - exiting")
        return

    # Start WebSocket server on port 8998 (matches Cloudflare tunnel)
    port = 8998
    logger.info(f"Starting WebSocket server on port {port}...")

    async with websockets.serve(
        lambda ws, path: audio_handler(ws, path, server),
        "0.0.0.0",
        port,
        max_size=10_000_000,
        ping_interval=20,
        ping_timeout=10
    ):
        logger.info(f"✅ Server running on ws://0.0.0.0:{port}")
        logger.info("\n" + "="*60)
        logger.info("CLOUDFLARE TUNNEL SETUP:")
        logger.info("1. Cloudflare tunnel should be running in another cell")
        logger.info("2. Copy the HTTPS URL from Cloudflare output")
        logger.info("3. Convert to WebSocket: https://xxx.trycloudflare.com")
        logger.info("   → wss://xxx.trycloudflare.com/audio")
        logger.info("4. Update ESP32 code with this URL")
        logger.info("="*60 + "\n")
        logger.info("Waiting for ESP32 connections...")

        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())
