from pymavlink import mavutil
from typing import Optional


# ============================================================================
# CONNECTION MANAGER
# ============================================================================

class ConnectionManager:
    """Handles connections to flight controller and UDP output"""
    
    def __init__(self, serial_port: str, baud_rate: int, udp_ip: str, udp_port: int):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.master: Optional[mavutil.mavfile] = None
        self.udp_output: Optional[mavutil.mavfile] = None
    
    def connect_to_fc(self) -> bool:
        """Connect to the flight controller via UART"""
        print(f"Connecting to {self.serial_port} at {self.baud_rate} baud...")
        
        try:
            self.master = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baud_rate,
                source_system=255
            )
            
            print("Waiting for heartbeat from flight controller...")
            self.master.wait_heartbeat()
            
            print(f"Heartbeat received from system {self.master.target_system}, "
                  f"component {self.master.target_component}")
            return True
            
        except Exception as e:
            print(f"Error connecting to flight controller: {e}")
            return False
    
    def create_udp_output(self) -> bool:
        """Add UDP output for broadcasting telemetry"""
        print(f"Creating UDP broadcast link to {self.udp_ip}:{self.udp_port}...")
        
        try:
            self.udp_output = mavutil.mavlink_connection(
                f'udpout:{self.udp_ip}:{self.udp_port}',
                source_system=self.master.target_system
            )
            
            print("UDP broadcast link created successfully")
            return True
            
        except Exception as e:
            print(f"Error creating UDP output: {e}")
            return False
    
    def close(self):
        """Close all connections"""
        if self.master:
            self.master.close()
        if self.udp_output:
            self.udp_output.close()