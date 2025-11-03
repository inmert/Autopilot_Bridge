"""
MAVSDK Drone Controller - Main Entry Point
Modular system for controlling drone via serial connection
"""

import asyncio
import logging
from drone_controller import DroneController
from config import Config

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def main():
    """Main entry point for the drone controller system"""
    
    # Initialize controller
    controller = DroneController(
        serial_port=Config.SERIAL_PORT,
        baud_rate=Config.BAUD_RATE
    )
    
    try:
        # Connect to drone
        logger.info("Connecting to drone...")
        await controller.connect()
        
        # Wait for drone to be ready
        logger.info("Waiting for drone to be ready...")
        await controller.wait_for_ready()
        
        logger.info("Drone is ready! System running...")
        
        # Keep the system running
        # In the future, this will handle Unity commands
        while controller.is_connected:
            await asyncio.sleep(1)
            
            # You can add periodic status checks here
            # telemetry = await controller.get_telemetry()
            # logger.info(f"Altitude: {telemetry.get('altitude')}m")
            
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    except Exception as e:
        logger.error(f"Error in main loop: {e}")
    finally:
        await controller.disconnect()


if __name__ == "__main__":
    asyncio.run(main())