"""
MAVLink Broadcaster Module
Broadcasts MAVLink messages from serial port to multiple UDP endpoints
Similar to MAVProxy forwarding functionality
"""

import asyncio
import logging
from typing import List, Tuple
from pymavlink import mavutil

logger = logging.getLogger(__name__)


class BroadcastEndpoint:
    """Represents a UDP broadcast endpoint"""
    
    def __init__(self, host: str, port: int):
        """
        Initialize broadcast endpoint
        
        Args:
            host: Target host/IP address
            port: Target UDP port
        """
        self.host = host
        self.port = port
        self.address = (host, port)
    
    def __str__(self):
        return f"udp:{self.host}:{self.port}"


class MAVLinkBroadcaster:
    """
    Broadcasts MAVLink messages from serial connection to multiple UDP endpoints
    Allows tools like QGroundControl, Mission Planner, or Unity to receive telemetry
    """
    
    def __init__(self, serial_port: str, baud_rate: int):
        """
        Initialize MAVLink broadcaster
        
        Args:
            serial_port: Serial port path (e.g., '/dev/ttyTHS1')
            baud_rate: Serial baud rate (e.g., 57600)
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.endpoints: List[BroadcastEndpoint] = []
        self.master = None
        self._broadcasting = False
        self._broadcast_task = None
        
        logger.info(f"MAVLink Broadcaster initialized for {serial_port}@{baud_rate}")
    
    def add_endpoint(self, host: str, port: int):
        """
        Add a UDP broadcast endpoint
        
        Args:
            host: Target host/IP address (e.g., '192.168.50.17' or 'localhost')
            port: Target UDP port (e.g., 14551)
        """
        endpoint = BroadcastEndpoint(host, port)
        self.endpoints.append(endpoint)
        logger.info(f"Added broadcast endpoint: {endpoint}")
    
    def add_endpoints(self, endpoints: List[Tuple[str, int]]):
        """
        Add multiple UDP broadcast endpoints
        
        Args:
            endpoints: List of (host, port) tuples
        """
        for host, port in endpoints:
            self.add_endpoint(host, port)
    
    async def start(self):
        """Start broadcasting MAVLink messages"""
        if self._broadcasting:
            logger.warning("Broadcaster already running")
            return
        
        if not self.endpoints:
            logger.warning("No broadcast endpoints configured")
            return
        
        try:
            # Create MAVLink connection
            connection_string = f"{self.serial_port},{self.baud_rate}"
            logger.info(f"Connecting to MAVLink on {connection_string}")
            
            # Create master connection (serial input)
            self.master = mavutil.mavlink_connection(
                connection_string,
                baud=self.baud_rate,
                source_system=255
            )
            
            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            logger.info(f"Heartbeat from system {self.master.target_system}, component {self.master.target_component}")
            
            # Create UDP output connections
            self.outputs = []
            for endpoint in self.endpoints:
                output = mavutil.mavlink_connection(
                    f"udpout:{endpoint.host}:{endpoint.port}",
                    source_system=255
                )
                self.outputs.append(output)
                logger.info(f"Created output connection to {endpoint}")
            
            # Start broadcasting
            self._broadcasting = True
            self._broadcast_task = asyncio.create_task(self._broadcast_loop())
            
            logger.info(f"Broadcasting started to {len(self.endpoints)} endpoint(s)")
            
        except Exception as e:
            logger.error(f"Failed to start broadcaster: {e}")
            raise
    
    async def _broadcast_loop(self):
        """Main broadcast loop - forwards messages from serial to UDP endpoints"""
        logger.info("Broadcast loop started")
        
        try:
            while self._broadcasting:
                # Receive message from serial (non-blocking with timeout)
                msg = self.master.recv_match(blocking=False, timeout=0.01)
                
                if msg is not None:
                    # Forward to all UDP endpoints
                    for output in self.outputs:
                        try:
                            output.mav.send(msg)
                        except Exception as e:
                            logger.error(f"Error forwarding to {output}: {e}")
                
                # Small delay to prevent CPU spinning
                await asyncio.sleep(0.001)
                
        except Exception as e:
            logger.error(f"Error in broadcast loop: {e}")
        finally:
            logger.info("Broadcast loop stopped")
    
    async def stop(self):
        """Stop broadcasting"""
        if not self._broadcasting:
            return
        
        logger.info("Stopping broadcaster...")
        self._broadcasting = False
        
        # Wait for broadcast task to complete
        if self._broadcast_task:
            self._broadcast_task.cancel()
            try:
                await self._broadcast_task
            except asyncio.CancelledError:
                pass
        
        # Close connections
        if self.master:
            self.master.close()
            self.master = None
        
        if hasattr(self, 'outputs'):
            for output in self.outputs:
                output.close()
            self.outputs.clear()
        
        logger.info("Broadcaster stopped")
    
    def is_broadcasting(self) -> bool:
        """Check if broadcaster is currently running"""
        return self._broadcasting
    
    def get_endpoints(self) -> List[str]:
        """Get list of configured endpoints as strings"""
        return [str(endpoint) for endpoint in self.endpoints]


class BroadcasterManager:
    """
    Manages MAVLink broadcaster lifecycle
    Provides easy integration with the main drone controller system
    """
    
    def __init__(self, serial_port: str, baud_rate: int):
        """
        Initialize broadcaster manager
        
        Args:
            serial_port: Serial port path
            baud_rate: Serial baud rate
        """
        self.broadcaster = MAVLinkBroadcaster(serial_port, baud_rate)
        self._running = False
    
    def configure_standard_endpoints(self, 
                                     remote_ip: str = "192.168.50.17",
                                     remote_port: int = 14551,
                                     local_port: int = 14551):
        """
        Configure standard broadcast endpoints (remote + localhost)
        
        Args:
            remote_ip: Remote IP address for GCS or Unity
            remote_port: Remote UDP port
            local_port: Local UDP port
        """
        self.broadcaster.add_endpoint(remote_ip, remote_port)
        self.broadcaster.add_endpoint("localhost", local_port)
        logger.info(f"Configured standard endpoints: {remote_ip}:{remote_port}, localhost:{local_port}")
    
    async def start(self):
        """Start the broadcaster"""
        if self._running:
            logger.warning("Broadcaster manager already running")
            return
        
        await self.broadcaster.start()
        self._running = True
        logger.info("Broadcaster manager started")
    
    async def stop(self):
        """Stop the broadcaster"""
        if not self._running:
            return
        
        await self.broadcaster.stop()
        self._running = False
        logger.info("Broadcaster manager stopped")
    
    def is_running(self) -> bool:
        """Check if broadcaster is running"""
        return self._running and self.broadcaster.is_broadcasting()


# Example usage
async def example_usage():
    """Example of how to use the MAVLink broadcaster"""
    
    # Create broadcaster
    broadcaster = MAVLinkBroadcaster('/dev/ttyTHS1', 57600)
    
    # Add endpoints (equivalent to MAVProxy --out parameters)
    broadcaster.add_endpoint('192.168.50.17', 14551)
    broadcaster.add_endpoint('localhost', 14551)
    
    # Or add multiple at once
    # broadcaster.add_endpoints([
    #     ('192.168.50.17', 14551),
    #     ('localhost', 14551),
    #     ('192.168.50.100', 14552)
    # ])
    
    # Start broadcasting
    await broadcaster.start()
    
    try:
        # Keep running
        while broadcaster.is_broadcasting():
            await asyncio.sleep(1)
            logger.info("Broadcasting active...")
    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        await broadcaster.stop()


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Run example
    asyncio.run(example_usage())