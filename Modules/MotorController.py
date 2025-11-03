
from pymavlink import mavutil
import time
import threading
from typing import Optional

# ============================================================================
# MOTOR CONTROLLER
# ============================================================================

class MotorController:
    """Handles all motor control operations"""
    
    def __init__(self, master: mavutil.mavfile):
        self.master = master
        self.last_rc_time = 0
        self.min_rc_interval = 1.0 / RC_OVERRIDE_RATE
        
        # Safe default RC values (centered controls, low throttle)
        # This prevents failsafe when no commands are being sent
        self.last_rc_values = [1500, 1500, 1100, 1500]  # Roll, Pitch, Throttle(low), Yaw
        self.rc_override_active = False
        self.rc_override_enabled = False  # Whether Unity has enabled RC control
        
        # Thread for continuous RC override sending
        self.rc_thread = None
        self.rc_thread_running = False
    
    def enable_rc_override(self):
        """Enable RC override mode and start continuous sending"""
        if not self.rc_override_enabled:
            self.rc_override_enabled = True
            self.rc_override_active = True
            
            # Start continuous RC override thread
            if self.rc_thread is None or not self.rc_thread.is_alive():
                self.rc_thread_running = True
                self.rc_thread = threading.Thread(target=self._rc_override_loop, daemon=True)
                self.rc_thread.start()
                print("RC override enabled - continuous sending started")
    
    def disable_rc_override(self):
        """Disable RC override mode"""
        self.rc_override_enabled = False
        self.rc_thread_running = False
        
        # Wait for thread to stop
        if self.rc_thread and self.rc_thread.is_alive():
            self.rc_thread.join(timeout=1.0)
        
        # Send clear command
        self.clear_rc_override()
        print("RC override disabled")
    
    def _rc_override_loop(self):
        """Continuously send RC override commands to prevent failsafe"""
        print("RC override loop started")
        
        while self.rc_thread_running and self.rc_override_enabled:
            try:
                current_time = time.time()
                
                # Always send RC override at the configured rate
                # This prevents ArduPilot from timing out and triggering failsafe
                self._send_rc_override(*self.last_rc_values)
                
                # Sleep for the interval
                time.sleep(self.min_rc_interval)
                
            except Exception as e:
                print(f"Error in RC override loop: {e}")
                time.sleep(0.1)
        
        print("RC override loop stopped")
    
    def set_rc_channels(self, throttle: int, yaw: int, pitch: int, roll: int) -> bool:
        """
        Update RC channel values from Unity joystick
        Values will be sent by the continuous loop
        """
        # Validate all values
        if not all(self._validate_pwm(val) for val in [throttle, yaw, pitch, roll]):
            print(f"Invalid RC values: T={throttle} Y={yaw} P={pitch} R={roll}")
            return False
        
        # Update the values that will be continuously sent
        self.last_rc_values = [roll, pitch, throttle, yaw]
        
        # Enable RC override if not already enabled
        if not self.rc_override_enabled:
            self.enable_rc_override()
        
        return True
    
    def _send_rc_override(self, roll: int, pitch: int, throttle: int, yaw: int):
        """Internal method to send RC override command"""
        # ArduCopter RC channel mapping:
        # 1 = Roll, 2 = Pitch, 3 = Throttle, 4 = Yaw, 5-8 = No override
        rc_channels = [
            roll,      # Channel 1
            pitch,     # Channel 2
            throttle,  # Channel 3
            yaw,       # Channel 4
            65535,     # Channel 5 - No override
            65535,     # Channel 6 - No override
            65535,     # Channel 7 - No override
            65535      # Channel 8 - No override
        ]
        
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channels
            )
            self.rc_override_active = True
            
        except Exception as e:
            print(f"Error sending RC override: {e}")
    
    def clear_rc_override(self):
        """Clear all RC overrides"""
        rc_channels = [65535] * 8
        
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channels
            )
            
            self.rc_override_active = False
            self.last_rc_values = [1500, 1500, 1100, 1500]  # Reset to safe defaults
            print("RC override cleared - returning control to transmitter")
            
        except Exception as e:
            print(f"Error clearing RC override: {e}")
    
    def set_motor_speed(self, motor_num: int, pwm_value: int) -> bool:
        """Set individual motor speed using RC override"""
        if not self._validate_motor_num(motor_num):
            return False
        
        if not self._validate_pwm(pwm_value):
            return False
        
        rc_channels = [65535] * 8
        rc_channels[motor_num - 1] = pwm_value
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channels
        )
        
        print(f"Motor {motor_num} set to PWM {pwm_value}")
        return True
    
    def set_all_motors(self, pwm_value: int) -> bool:
        """Set all motors to the same speed"""
        if not self._validate_pwm(pwm_value):
            return False
        
        rc_channels = [pwm_value] * 8
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            *rc_channels
        )
        
        print(f"All motors set to PWM {pwm_value}")
        return True
    
    def motor_test(self, motor_num: int, throttle_percent: int, duration_sec: int = 2) -> bool:
        """Test individual motor"""
        if not self._validate_motor_num(motor_num):
            return False
        
        if not self._validate_throttle_percent(throttle_percent):
            return False
        
        # Disable RC override before motor test
        if self.rc_override_enabled:
            print("Disabling RC override for motor test...")
            self.disable_rc_override()
            time.sleep(0.1)
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,
            motor_num,
            0,  # MOTOR_TEST_THROTTLE_PERCENT = 0
            throttle_percent,
            duration_sec,
            0,  # Motor count (0 = one motor)
            0,  # Test order
            0   # Empty
        )
        
        print(f"Motor {motor_num} test: {throttle_percent}% for {duration_sec} seconds")
        return True
    
    def motor_test_all(self, throttle_percent: int, duration_sec: int = 2) -> bool:
        """Test all motors in sequence"""
        if not self._validate_throttle_percent(throttle_percent):
            return False
        
        # Disable RC override before motor test
        if self.rc_override_enabled:
            print("Disabling RC override for motor test...")
            self.disable_rc_override()
            time.sleep(0.1)
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,
            1,  # Start with motor 1
            0,  # MOTOR_TEST_THROTTLE_PERCENT = 0
            throttle_percent,
            duration_sec,
            4,  # Motor count (4 motors for quadcopter)
            1,  # Test order: 1 = sequence
            0   # Empty
        )
        
        print(f"Testing all motors: {throttle_percent}% for {duration_sec} seconds each")
        return True
    
    def get_rc_override_status(self) -> dict:
        """Get current RC override status"""
        return {
            'active': self.rc_override_active,
            'enabled': self.rc_override_enabled,
            'roll': self.last_rc_values[0],
            'pitch': self.last_rc_values[1],
            'throttle': self.last_rc_values[2],
            'yaw': self.last_rc_values[3]
        }
    
    @staticmethod
    def _validate_motor_num(motor_num: int) -> bool:
        if motor_num < 1 or motor_num > 8:
            print(f"Invalid motor number: {motor_num}. Must be 1-8")
            return False
        return True
    
    @staticmethod
    def _validate_pwm(pwm_value: int) -> bool:
        if pwm_value < 1000 or pwm_value > 2000:
            print(f"Invalid PWM value: {pwm_value}. Must be 1000-2000")
            return False
        return True
    
    @staticmethod
    def _validate_throttle_percent(throttle_percent: int) -> bool:
        if throttle_percent < 0 or throttle_percent > 100:
            print(f"Invalid throttle: {throttle_percent}. Must be 0-100%")
            return False
        return True
