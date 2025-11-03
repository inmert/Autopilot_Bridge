"""
Configuration settings for the drone controller system
"""


class Config:
    """Central configuration class"""
    
    # Serial connection settings
    SERIAL_PORT = '/dev/ttyTHS1'
    BAUD_RATE = 57600
    
    # Connection timeout (seconds)
    CONNECTION_TIMEOUT = 30
    
    # Telemetry update rate (Hz)
    TELEMETRY_RATE = 10
    
    # Safety settings
    MAX_ALTITUDE = 100  # meters
    MIN_BATTERY = 20    # percent
    
    # Command timeout (seconds)
    COMMAND_TIMEOUT = 10
    
    # Logging
    LOG_LEVEL = 'INFO'
    LOG_FILE = 'drone_controller.log'