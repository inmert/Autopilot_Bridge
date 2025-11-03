"""
Command handler module for processing commands from Unity or other sources
This module will be the interface between Unity and the drone controller
"""

import asyncio
import logging
from enum import Enum
from typing import Dict, Any, Optional
from drone_controller import DroneController


logger = logging.getLogger(__name__)


class CommandType(Enum):
    """Available command types"""
    ARM = "arm"
    DISARM = "disarm"
    TAKEOFF = "takeoff"
    LAND = "land"
    GOTO = "goto"
    RTL = "return_to_launch"
    GET_TELEMETRY = "get_telemetry"
    SET_SPEED = "set_speed"
    HOLD = "hold"


class CommandHandler:
    """Handles incoming commands and executes them on the drone"""
    
    def __init__(self, controller: DroneController):
        """
        Initialize command handler
        
        Args:
            controller: DroneController instance
        """
        self.controller = controller
        self.command_queue = asyncio.Queue()
        self._processing = False
        
        # Map command types to handler methods
        self.command_map = {
            CommandType.ARM: self._handle_arm,
            CommandType.DISARM: self._handle_disarm,
            CommandType.TAKEOFF: self._handle_takeoff,
            CommandType.LAND: self._handle_land,
            CommandType.GOTO: self._handle_goto,
            CommandType.RTL: self._handle_rtl,
            CommandType.GET_TELEMETRY: self._handle_get_telemetry,
        }
    
    async def start_processing(self):
        """Start processing commands from the queue"""
        if self._processing:
            logger.warning("Command processing already started")
            return
        
        self._processing = True
        logger.info("Starting command processing...")
        
        while self._processing:
            try:
                # Wait for command with timeout to allow checking _processing flag
                command = await asyncio.wait_for(
                    self.command_queue.get(), 
                    timeout=1.0
                )
                await self._execute_command(command)
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                logger.error(f"Error processing command: {e}")
    
    async def stop_processing(self):
        """Stop processing commands"""
        logger.info("Stopping command processing...")
        self._processing = False
    
    async def add_command(self, command_type: str, params: Optional[Dict[str, Any]] = None):
        """
        Add a command to the queue
        
        Args:
            command_type: Type of command (from CommandType enum)
            params: Optional parameters for the command
        """
        command = {
            'type': command_type,
            'params': params or {}
        }
        await self.command_queue.put(command)
        logger.info(f"Command queued: {command_type}")
    
    async def _execute_command(self, command: Dict[str, Any]):
        """
        Execute a command
        
        Args:
            command: Command dictionary with 'type' and 'params'
        """
        command_type_str = command.get('type')
        params = command.get('params', {})
        
        try:
            # Convert string to CommandType enum
            command_type = CommandType(command_type_str)
            
            # Get handler function
            handler = self.command_map.get(command_type)
            
            if handler:
                logger.info(f"Executing command: {command_type_str}")
                result = await handler(params)
                logger.info(f"Command completed: {command_type_str}")
                return result
            else:
                logger.warning(f"No handler for command type: {command_type_str}")
                
        except ValueError:
            logger.error(f"Invalid command type: {command_type_str}")
        except Exception as e:
            logger.error(f"Error executing command {command_type_str}: {e}")
    
    # Command handler methods
    
    async def _handle_arm(self, params: Dict[str, Any]):
        """Handle arm command"""
        await self.controller.arm()
    
    async def _handle_disarm(self, params: Dict[str, Any]):
        """Handle disarm command"""
        await self.controller.disarm()
    
    async def _handle_takeoff(self, params: Dict[str, Any]):
        """Handle takeoff command"""
        altitude = params.get('altitude', 2.5)
        await self.controller.takeoff(altitude)
    
    async def _handle_land(self, params: Dict[str, Any]):
        """Handle land command"""
        await self.controller.land()
    
    async def _handle_goto(self, params: Dict[str, Any]):
        """Handle goto command"""
        latitude = params.get('latitude')
        longitude = params.get('longitude')
        altitude = params.get('altitude')
        yaw = params.get('yaw', float('nan'))
        
        if latitude is None or longitude is None or altitude is None:
            logger.error("Goto command missing required parameters")
            return
        
        await self.controller.goto_location(latitude, longitude, altitude, yaw)
    
    async def _handle_rtl(self, params: Dict[str, Any]):
        """Handle return to launch command"""
        await self.controller.return_to_launch()
    
    async def _handle_get_telemetry(self, params: Dict[str, Any]):
        """Handle get telemetry command"""
        telemetry = await self.controller.get_telemetry()
        return telemetry


# Example usage for Unity integration (to be implemented later)
class UnityCommandInterface:
    """
    Interface for receiving commands from Unity
    This is a placeholder for future Unity integration
    """
    
    def __init__(self, command_handler: CommandHandler):
        """
        Initialize Unity interface
        
        Args:
            command_handler: CommandHandler instance
        """
        self.command_handler = command_handler
    
    async def process_unity_message(self, message: str):
        """
        Process a message from Unity
        
        Args:
            message: JSON string from Unity with command and parameters
        """
        # TODO: Parse Unity message format
        # TODO: Convert to command and add to queue
        # Example:
        # data = json.loads(message)
        # await self.command_handler.add_command(data['command'], data['params'])
        pass