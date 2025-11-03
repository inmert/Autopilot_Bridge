"""
Telemetry monitoring module for continuous drone state tracking
"""

import asyncio
import logging
from typing import Callable, Optional
from mavsdk import System

logger = logging.getLogger(__name__)


class TelemetryMonitor:
    """Monitors and streams telemetry data from the drone"""
    
    def __init__(self, drone: System):
        """
        Initialize telemetry monitor
        
        Args:
            drone: MAVSDK System instance
        """
        self.drone = drone
        self._monitoring = False
        self._tasks = []
    
    async def start_monitoring(self, 
                              position_callback: Optional[Callable] = None,
                              battery_callback: Optional[Callable] = None,
                              attitude_callback: Optional[Callable] = None,
                              gps_callback: Optional[Callable] = None):
        """
        Start monitoring telemetry streams
        
        Args:
            position_callback: Function to call with position updates
            battery_callback: Function to call with battery updates
            attitude_callback: Function to call with attitude updates
            gps_callback: Function to call with GPS updates
        """
        if self._monitoring:
            logger.warning("Telemetry monitoring already started")
            return
        
        self._monitoring = True
        logger.info("Starting telemetry monitoring...")
        
        # Start monitoring tasks
        if position_callback:
            task = asyncio.create_task(self._monitor_position(position_callback))
            self._tasks.append(task)
        
        if battery_callback:
            task = asyncio.create_task(self._monitor_battery(battery_callback))
            self._tasks.append(task)
        
        if attitude_callback:
            task = asyncio.create_task(self._monitor_attitude(attitude_callback))
            self._tasks.append(task)
        
        if gps_callback:
            task = asyncio.create_task(self._monitor_gps(gps_callback))
            self._tasks.append(task)
        
        logger.info(f"Started {len(self._tasks)} telemetry monitoring tasks")
    
    async def _monitor_position(self, callback: Callable):
        """Monitor position updates"""
        try:
            async for position in self.drone.telemetry.position():
                if not self._monitoring:
                    break
                await callback(position)
        except Exception as e:
            logger.error(f"Error in position monitoring: {e}")
    
    async def _monitor_battery(self, callback: Callable):
        """Monitor battery updates"""
        try:
            async for battery in self.drone.telemetry.battery():
                if not self._monitoring:
                    break
                await callback(battery)
        except Exception as e:
            logger.error(f"Error in battery monitoring: {e}")
    
    async def _monitor_attitude(self, callback: Callable):
        """Monitor attitude updates"""
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                if not self._monitoring:
                    break
                await callback(attitude)
        except Exception as e:
            logger.error(f"Error in attitude monitoring: {e}")
    
    async def _monitor_gps(self, callback: Callable):
        """Monitor GPS updates"""
        try:
            async for gps_info in self.drone.telemetry.gps_info():
                if not self._monitoring:
                    break
                await callback(gps_info)
        except Exception as e:
            logger.error(f"Error in GPS monitoring: {e}")
    
    async def stop_monitoring(self):
        """Stop all telemetry monitoring"""
        if not self._monitoring:
            return
        
        logger.info("Stopping telemetry monitoring...")
        self._monitoring = False
        
        # Cancel all tasks
        for task in self._tasks:
            task.cancel()
        
        # Wait for tasks to complete
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()
        
        logger.info("Telemetry monitoring stopped")