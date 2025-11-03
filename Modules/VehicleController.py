from pymavlink import mavutil
from typing import Optional
from Modules.ConnectionManager import ConnectionManager

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