"""
CASE Personaplex WebSocket Server
Audio-to-audio AI conversation using Nvidia Personaplex (Moshi)

This server receives audio from the ESP32 robot, processes it through
Personaplex, and sends back audio responses.

Setup Instructions:
1. Run this in Google Colab with GPU runtime
2. Install dependencies (see requirements.txt)
3. Accept Personaplex license on Hugging Face
4. Start ngrok tunnel for public access
"""

import asyncio
import websockets
import numpy as np
import io
import struct
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
            # from moshi.server import PersonaplexModel
            # self.model = PersonaplexModel(
            #     voice_prompt=self.voice_model,
            #     system_prompt=self.get_case_persona()
            # )

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

    # Start WebSocket server
    port = 8765
    logger.info(f"Starting WebSocket server on port {port}...")

    async with websockets.serve(
        lambda ws, path: audio_handler(ws, path, server),
        "0.0.0.0",
        port,
        max_size=10_000_000,  # 10MB max message size
        ping_interval=20,
        ping_timeout=10
    ):
        logger.info(f"Server running on ws://0.0.0.0:{port}")
        logger.info("Waiting for ESP32 connections...")
        logger.info("\nNext steps:")
        logger.info("1. Start ngrok: !ngrok http 8765")
        logger.info("2. Copy ngrok URL to ESP32 code")
        logger.info("3. Upload code to ESP32")

        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())
