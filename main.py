#!/usr/bin/env python3
"""
ArduPilot UART to UDP Bridge with Unity TCP Command Interface
Includes RC override support for joystick control
"""

from pymavlink import mavutil
import time
import sys
import threading
import socket
from typing import Optional
from Modules.ConnectionManager import ConnectionManager
from Modules.MotorController import MotorController

# ============================================================================
# CONFIGURATION
# ============================================================================


SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 57600
#UDP_IP = '192.168.50.17'
UDP_IP = '10.42.0.60'
UDP_PORT = 14550
TCP_COMMAND_PORT = 5760

# RC Override configuration
RC_OVERRIDE_RATE = 25  # Hz - Send at 25Hz to stay well above ArduPilot's timeout
RC_OVERRIDE_TIMEOUT = 0.5  # seconds - If no RC command for this long, send safe defaults




# ============================================================================
# VEHICLE CONTROLLER
# ============================================================================

class VehicleController:
    """Handles vehicle arming, disarming, and mode changes"""
    
    def __init__(self, master: mavutil.mavfile):
        self.master = master
    
    def arm(self):
        """Arm the vehicle"""
        print("Arming vehicle...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Vehicle armed!")
    
    def disarm(self):
        """Disarm the vehicle"""
        print("Disarming vehicle...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("Vehicle disarmed!")
    
    def set_mode(self, mode: str):
        """Set flight mode"""
        mode_id = self.master.mode_mapping()[mode]
        self.master.set_mode(mode_id)
        print(f"Mode set to {mode}")
    
    def disable_prearm_checks(self):
        """Disable pre-arm safety checks"""
        print("Disabling pre-arm checks...")
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'ARMING_CHECK',
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        print("Pre-arm checks disabled. WARNING: Use with extreme caution!")
    
    def enable_prearm_checks(self):
        """Re-enable pre-arm safety checks"""
        print("Enabling pre-arm checks...")
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'ARMING_CHECK',
            1,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        print("Pre-arm checks enabled")

# ============================================================================
# MESSAGE HANDLER
# ============================================================================

class MessageHandler:
    """Handles message receiving and forwarding"""
    
    def __init__(self, master: mavutil.mavfile, udp_output: Optional[mavutil.mavfile]):
        self.master = master
        self.udp_output = udp_output
        self.running = False
        self.message_count = 0
    
    def start(self):
        """Start the message forwarding loop"""
        self.running = True
        print("\nStarting message forwarding loop...")
        
        try:
            while self.running:
                msg = self.master.recv_match(blocking=False, timeout=0.01)
                
                if msg is not None:
                    self.message_count += 1
                    
                    if self.udp_output:
                        self.udp_output.write(msg.get_msgbuf())
                    
                    self._process_message(msg)
                
                time.sleep(0.001)
        
        except Exception as e:
            print(f"Error in message loop: {e}")
        
        finally:
            print(f"\nTotal messages forwarded: {self.message_count}")
    
    def stop(self):
        """Stop the message forwarding loop"""
        self.running = False
    
    def _process_message(self, msg):
        """Process and display interesting messages"""
        msg_type = msg.get_type()
        
        if msg_type == 'HEARTBEAT':
            if self.message_count % 100 == 0:
                mode = mavutil.mode_string_v10(msg)
                armed = "ARMED" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
                print(f"[{self.message_count}] Mode: {mode}, Status: {armed}")
        
        elif msg_type == 'STATUSTEXT':
            print(f"[{self.message_count}] AP: {msg.text}")
        
        elif msg_type == 'SERVO_OUTPUT_RAW':
            if self.message_count % 100 == 0:
                print(f"[{self.message_count}] Motors: {msg.servo1_raw}, "
                      f"{msg.servo2_raw}, {msg.servo3_raw}, {msg.servo4_raw}")

# ============================================================================
# TCP COMMAND SERVER
# ============================================================================

class TCPCommandServer:
    """TCP server to receive commands from Unity"""
    
    def __init__(self, vehicle, motors: MotorController, port: int):
        self.vehicle = vehicle
        self.motors = motors
        self.port = port
        self.running = False
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
    
    def start(self):
        """Start the TCP server"""
        self.running = True
        
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            self.server_socket.bind(('0.0.0.0', self.port))
            self.server_socket.listen(1)
            print(f"TCP command server listening on port {self.port}")
            
            while self.running:
                print("Waiting for Unity connection...")
                self.client_socket, client_address = self.server_socket.accept()
                print(f"Unity connected from {client_address}")
                
                # Enable RC override immediately on connection with safe defaults
                print("Enabling RC override to clear failsafe...")
                self.motors.enable_rc_override()
                
                self._send_response("CONNECTED")
                self._handle_client()
        
        except Exception as e:
            print(f"TCP server error: {e}")
        
        finally:
            self._cleanup()
    
    def stop(self):
        """Stop the TCP server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
    
    def _send_response(self, message: str):
        """Send response back to Unity"""
        if self.client_socket:
            try:
                self.client_socket.send(f"{message}\n".encode('utf-8'))
            except Exception as e:
                print(f"Error sending response: {e}")
    
    def _handle_client(self):
        """Handle commands from connected Unity client"""
        buffer = ""
        
        try:
            while self.running:
                data = self.client_socket.recv(1024).decode('utf-8')
                
                if not data:
                    print("Unity disconnected")
                    break
                
                buffer += data
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    command = line.strip()
                    
                    if command:
                        self._process_command(command)
        
        except Exception as e:
            print(f"Client handling error: {e}")
        
        finally:
            # Disable RC override when Unity disconnects
            print("Unity disconnected - disabling RC override...")
            self.motors.disable_rc_override()
            
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
    
    def _process_command(self, command_str: str):
        """Process a command from Unity"""
        try:
            parts = command_str.split()
            
            if not parts:
                return
            
            cmd = parts[0].lower()
            success = False
            response = "ERROR"
            
            # RC override for joystick control
            if cmd == 'rc' and len(parts) == 5:
                throttle = int(parts[1])
                yaw = int(parts[2])
                pitch = int(parts[3])
                roll = int(parts[4])
                
                success = self.motors.set_rc_channels(throttle, yaw, pitch, roll)
                # Don't send response for RC commands to avoid flooding
                return
            
            # Clear RC override
            elif cmd == 'clear':
                self.motors.disable_rc_override()
                success = True
                response = "OK CLEAR"
            
            # Vehicle control commands
            elif cmd == 'arm':
                self.vehicle.arm()
                success = True
                response = "OK ARM"
            
            elif cmd == 'disarm':
                self.vehicle.disarm()
                success = True
                response = "OK DISARM"
            
            elif cmd == 'mode' and len(parts) == 2:
                self.vehicle.set_mode(parts[1].upper())
                success = True
                response = f"OK MODE {parts[1].upper()}"
            
            # Motor control commands
            elif cmd == 'motor' and len(parts) == 3:
                motor_num = int(parts[1])
                pwm_value = int(parts[2])
                success = self.motors.set_motor_speed(motor_num, pwm_value)
                response = f"OK MOTOR {motor_num} {pwm_value}" if success else "ERROR MOTOR"
            
            elif cmd == 'all' and len(parts) == 2:
                pwm_value = int(parts[1])
                success = self.motors.set_all_motors(pwm_value)
                response = f"OK ALL {pwm_value}" if success else "ERROR ALL"
            
            # Motor test commands
            elif cmd == 'test' and len(parts) >= 3:
                motor_num = int(parts[1])
                throttle_pct = int(parts[2])
                duration = int(parts[3]) if len(parts) == 4 else 2
                success = self.motors.motor_test(motor_num, throttle_pct, duration)
                response = f"OK TEST {motor_num}" if success else "ERROR TEST"
            
            elif cmd == 'testall' and len(parts) >= 2:
                throttle_pct = int(parts[1])
                duration = int(parts[2]) if len(parts) == 3 else 2
                success = self.motors.motor_test_all(throttle_pct, duration)
                response = "OK TESTALL" if success else "ERROR TESTALL"
            
            # Safety commands
            elif cmd == 'noprearm':
                self.vehicle.disable_prearm_checks()
                success = True
                response = "OK NOPREARM"
            
            elif cmd == 'prearm':
                self.vehicle.enable_prearm_checks()
                success = True
                response = "OK PREARM"
            
            # Status query
            elif cmd == 'status':
                rc_status = self.motors.get_rc_override_status()
                response = f"OK STATUS {rc_status['enabled']} {rc_status['throttle']} {rc_status['yaw']} {rc_status['pitch']} {rc_status['roll']}"
                success = True
            
            else:
                print(f"Unknown command: {command_str}")
                response = f"ERROR UNKNOWN {cmd}"
            
            # Send response
            if success:
                self._send_response(response)
            else:
                self._send_response(response)
        
        except ValueError as e:
            error_msg = f"ERROR INVALID_FORMAT {command_str}"
            print(f"Invalid command format: {command_str} - {e}")
            self._send_response(error_msg)
        
        except Exception as e:
            error_msg = f"ERROR EXCEPTION {str(e)}"
            print(f"Error processing command: {e}")
            self._send_response(error_msg)
    
    def _cleanup(self):
        """Cleanup sockets"""
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

            
# ============================================================================
# MAIN BRIDGE CLASS
# ============================================================================

class ArduPilotBridge:
    """Main bridge coordinator"""
    
    def __init__(self):
        self.conn_manager = ConnectionManager(SERIAL_PORT, BAUD_RATE, UDP_IP, UDP_PORT)
        self.vehicle: Optional[VehicleController] = None
        self.motors: Optional[MotorController] = None
        self.msg_handler: Optional[MessageHandler] = None
        self.tcp_server: Optional[TCPCommandServer] = None
        self.message_thread: Optional[threading.Thread] = None
        self.tcp_thread: Optional[threading.Thread] = None
    
    def run(self):
        """Main run method"""
        print("=" * 60)
        print("ArduPilot UART to UDP Bridge with Unity Interface")
        print("=" * 60)
        print()
        
        if not self.conn_manager.connect_to_fc():
            return
        
        if not self.conn_manager.create_udp_output():
            return
        
        self.vehicle = VehicleController(self.conn_manager.master)
        self.motors = MotorController(self.conn_manager.master)
        self.msg_handler = MessageHandler(self.conn_manager.master, self.conn_manager.udp_output)
        self.tcp_server = TCPCommandServer(self.vehicle, self.motors, TCP_COMMAND_PORT)
        
        self.message_thread = threading.Thread(target=self.msg_handler.start, daemon=True)
        self.message_thread.start()
        
        self.tcp_thread = threading.Thread(target=self.tcp_server.start, daemon=False)
        self.tcp_thread.start()
        
        try:
            self.tcp_thread.join()
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        
        self._cleanup()
    
    def _cleanup(self):
        """Cleanup and shutdown"""
        if self.tcp_server:
            self.tcp_server.stop()
        
        if self.msg_handler:
            self.msg_handler.stop()
        
        time.sleep(0.5)
        
        if self.motors:
            self.motors.clear_rc_override()
        
        self.conn_manager.close()
        print("Shutdown complete")

# ============================================================================
# ENTRY POINT
# ============================================================================

def main():
    """Main function"""
    bridge = ArduPilotBridge()
    bridge.run()

if __name__ == "__main__":
    main()
