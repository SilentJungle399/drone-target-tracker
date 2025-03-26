from pymavlink import mavutil
import time
import json
import sys

config = json.load(open("settings.json"))["rpi" if sys.platform == "linux" else "sim"]

# Connect to the drone
master = mavutil.mavlink_connection(**config)
master.wait_heartbeat()
print(f"💓 Heartbeat received. System ID: {master.target_system}, Component ID: {master.target_component}")

# Ensure correct system & component IDs
master.target_system = 1
master.target_component = 1
print(f"✅ Fixed System ID: {master.target_system}, Fixed Component ID: {master.target_component}")

# Function to set parameters
def set_param(param_name, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        param_name.encode(), value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(f"🔧 Set {param_name} = {value}")
    time.sleep(0.5)  # Allow time for parameter change

# Disable safety features for testing
set_param("ARMING_CHECK", 0)
set_param("SIM_GPS_DISABLE", 0)
set_param("BRD_SAFETYENABLE", 0)
set_param("GPS_AUTO_CONFIG", 0)

time.sleep(2)  # Ensure params are applied before continuing

# Ensure the mode is correctly set
def set_mode(mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"✈️  {mode_name} mode set.")
    time.sleep(2)

# First switch to STABILIZE mode
set_mode("STABILIZE")

# Then switch to GUIDED mode
set_mode("GUIDED")

# Request arming
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("⚡ Arming command sent!")

# Wait for acknowledgment
while True:
    msg = master.recv_match(blocking=True, timeout=5)
    if msg:
        if msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == 0:
                print("✅ Arming successful!")
            else:
                print(f"❌ Arming failed with result code: {msg.result}")
            break
    else:
        print("⚠️ No ACK received, retrying...")
        time.sleep(2)
