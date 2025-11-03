"""
Main drone controller class using MAVSDK-Python
"""

import asyncio
import logging
from mavsdk import System
from mavsdk.telemetry import LandedState
from config import Config

logger = logging.getLogger(__name__)


class DroneController:
    """Main controller for drone operations"""
    
    def __init__(self, serial_port: str, baud_rate: int):
        """
        Initialize the drone controller
        
        Args:
            serial_port: Path to serial port (e.g., '/dev/ttyTHS1')
            baud_rate: Baud rate for serial connection (e.g., 57600)
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.drone = System()
        self.is_connected = False
        
        # Connection string for serial
        self.connection_string = f"serial://{serial_port}:{baud_rate}"
        
        logger.info(f"Drone controller initialized with {self.connection_string}")
    
    async def connect(self):
        """Connect to the drone via serial"""
        try:
            logger.info(f"Connecting to drone via {self.connection_string}")
            await self.drone.connect(system_address=self.connection_string)
            self.is_connected = True
            logger.info("Connection established")
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            raise
    
    async def wait_for_ready(self, timeout: int = Config.CONNECTION_TIMEOUT):
        """
        Wait for drone to be ready (connected and healthy)
        
        Args:
            timeout: Maximum time to wait in seconds
        """
        logger.info("Checking drone health...")
        
        try:
            # Wait for drone to be discovered
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    logger.info("Drone discovered!")
                    break
            
            # Wait for health checks
            logger.info("Waiting for health checks...")
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    logger.info("Drone is healthy and ready!")
                    break
                    
        except asyncio.TimeoutError:
            logger.error(f"Timeout waiting for drone to be ready ({timeout}s)")
            raise
        except Exception as e:
            logger.error(f"Error waiting for drone: {e}")
            raise
    
    async def get_telemetry(self) -> dict:
        """
        Get current telemetry data
        
        Returns:
            Dictionary with telemetry data
        """
        try:
            position = None
            battery = None
            gps = None
            attitude = None
            
            # Get position
            async for pos in self.drone.telemetry.position():
                position = pos
                break
            
            # Get battery
            async for bat in self.drone.telemetry.battery():
                battery = bat
                break
            
            # Get GPS info
            async for gps_info in self.drone.telemetry.gps_info():
                gps = gps_info
                break
            
            # Get attitude
            async for att in self.drone.telemetry.attitude_euler():
                attitude = att
                break
            
            return {
                'altitude': position.relative_altitude_m if position else None,
                'latitude': position.latitude_deg if position else None,
                'longitude': position.longitude_deg if position else None,
                'battery': battery.remaining_percent if battery else None,
                'gps_satellites': gps.num_satellites if gps else None,
                'roll': attitude.roll_deg if attitude else None,
                'pitch': attitude.pitch_deg if attitude else None,
                'yaw': attitude.yaw_deg if attitude else None
            }
        except Exception as e:
            logger.error(f"Error getting telemetry: {e}")
            return {}
    
    async def arm(self):
        """Arm the drone"""
        try:
            logger.info("Arming drone...")
            await self.drone.action.arm()
            logger.info("Drone armed")
        except Exception as e:
            logger.error(f"Failed to arm: {e}")
            raise
    
    async def disarm(self):
        """Disarm the drone"""
        try:
            logger.info("Disarming drone...")
            await self.drone.action.disarm()
            logger.info("Drone disarmed")
        except Exception as e:
            logger.error(f"Failed to disarm: {e}")
            raise
    
    async def takeoff(self, altitude: float = 2.5):
        """
        Takeoff to specified altitude
        
        Args:
            altitude: Target altitude in meters
        """
        try:
            logger.info(f"Taking off to {altitude}m...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            logger.info("Takeoff command sent")
        except Exception as e:
            logger.error(f"Failed to takeoff: {e}")
            raise
    
    async def land(self):
        """Land the drone"""
        try:
            logger.info("Landing...")
            await self.drone.action.land()
            logger.info("Land command sent")
        except Exception as e:
            logger.error(f"Failed to land: {e}")
            raise
    
    async def goto_location(self, latitude: float, longitude: float, 
                           altitude: float, yaw: float = float('nan')):
        """
        Go to specified GPS location
        
        Args:
            latitude: Target latitude in degrees
            longitude: Target longitude in degrees
            altitude: Target altitude in meters (relative)
            yaw: Target yaw in degrees (optional)
        """
        try:
            logger.info(f"Going to location: {latitude}, {longitude}, {altitude}m")
            await self.drone.action.goto_location(latitude, longitude, altitude, yaw)
            logger.info("Goto command sent")
        except Exception as e:
            logger.error(f"Failed to goto location: {e}")
            raise
    
    async def return_to_launch(self):
        """Return to launch position"""
        try:
            logger.info("Returning to launch...")
            await self.drone.action.return_to_launch()
            logger.info("RTL command sent")
        except Exception as e:
            logger.error(f"Failed to return to launch: {e}")
            raise
    
    async def disconnect(self):
        """Disconnect from the drone"""
        if self.is_connected:
            logger.info("Disconnecting from drone...")
            self.is_connected = False
            logger.info("Disconnected")