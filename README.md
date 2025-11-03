# MAVSDK Drone Controller System

A modular Python system for controlling drones via MAVSDK-Python with serial connection.

## Requirements

```
mavsdk==2.12.2
asyncio
```

## Installation

1. Install Python dependencies:
```bash
pip install mavsdk
```

2. Ensure you have proper permissions for serial port access:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

3. Verify serial port exists:
```bash
ls -l /dev/ttyTHS1
```

## Project Structure

```
drone_controller/
├── main.py                 # Main entry point
├── config.py              # Configuration settings
├── drone_controller.py    # Core drone controller
├── telemetry_monitor.py   # Telemetry monitoring
├── command_handler.py     # Command processing
└── requirements.txt       # Python dependencies
```

## Module Overview

### 1. **config.py**
- Centralized configuration
- Serial port settings
- Safety parameters
- Adjustable timeouts and rates

### 2. **drone_controller.py**
- Main drone control interface
- Connection management
- Basic flight commands (arm, takeoff, land, goto, RTL)
- Telemetry access

### 3. **telemetry_monitor.py**
- Continuous telemetry streaming
- Callback-based updates
- Position, battery, attitude, GPS monitoring
- Independent monitoring tasks

### 4. **command_handler.py**
- Command queue management
- Asynchronous command execution
- Unity interface placeholder
- Extensible command system

### 5. **main.py**
- System initialization
- Connection establishment
- Main event loop

## Usage

### Basic Usage

```bash
python main.py
```

### Testing Individual Components

```python
import asyncio
from drone_controller import DroneController
from config import Config

async def test():
    controller = DroneController(Config.SERIAL_PORT, Config.BAUD_RATE)
    await controller.connect()
    await controller.wait_for_ready()
    
    # Get telemetry
    telemetry = await controller.get_telemetry()
    print(f"Altitude: {telemetry['altitude']}m")
    print(f"Battery: {telemetry['battery']}%")
    
    await controller.disconnect()

asyncio.run(test())
```

### Using Command Handler

```python
from command_handler import CommandHandler, CommandType

# Initialize
handler = CommandHandler(controller)

# Add commands
await handler.add_command(CommandType.ARM.value)
await handler.add_command(CommandType.TAKEOFF.value, {'altitude': 5.0})
await handler.add_command(CommandType.LAND.value)

# Start processing
await handler.start_processing()
```

### Using Telemetry Monitor

```python
from telemetry_monitor import TelemetryMonitor

# Define callbacks
async def on_position(position):
    print(f"Position: {position.latitude_deg}, {position.longitude_deg}")

async def on_battery(battery):
    print(f"Battery: {battery.remaining_percent}%")

# Start monitoring
monitor = TelemetryMonitor(controller.drone)
await monitor.start_monitoring(
    position_callback=on_position,
    battery_callback=on_battery
)
```

## Configuration

Edit `config.py` to adjust:

- **Serial settings**: Port and baud rate
- **Safety limits**: Max altitude, min battery
- **Timeouts**: Connection and command timeouts
- **Telemetry rate**: Update frequency

## Future Unity Integration

The `UnityCommandInterface` class in `command_handler.py` is prepared for Unity integration. You'll need to:

1. Implement a communication protocol (TCP/UDP socket, WebSocket, etc.)
2. Parse Unity message format
3. Convert Unity commands to CommandType
4. Send telemetry data back to Unity

Example structure for Unity messages:
```json
{
    "command": "takeoff",
    "params": {
        "altitude": 5.0
    }
}
```

## Safety Features

- Connection health monitoring
- Timeout protection
- Battery level checking
- GPS status verification
- Proper error handling and logging

## Extending the System

### Adding New Commands

1. Add to `CommandType` enum in `command_handler.py`
2. Create handler method (`_handle_your_command`)
3. Add to `command_map` dictionary
4. Implement in `drone_controller.py` if needed

### Adding New Telemetry

1. Add monitoring method in `telemetry_monitor.py`
2. Create callback parameter in `start_monitoring()`
3. Add to monitoring tasks

## Troubleshooting

### Serial Port Access Denied
```bash
sudo chmod 666 /dev/ttyTHS1
# Or add user to dialout group (recommended)
```

### Connection Timeout
- Check FC is powered and connected
- Verify baud rate matches FC settings
- Check MAVLink is enabled on FC serial port

### No GPS Lock
- Ensure clear sky view
- Wait for sufficient satellites (usually 6+)
- Check FC GPS configuration


## Notes

- Always test in a safe environment
- Ensure proper calibration before flight
- Follow local drone regulations
- Maintain line of sight during operation
