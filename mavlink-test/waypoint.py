import json, time, sys
from pymavlink import mavutil

config = json.load(open("settings.json"))["rpi" if sys.platform == "linux" else "sim"]

def wait_for_ack(master, command):
    """Wait for an acknowledgment for a given command"""
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg and msg.command == command:
            print(f"✅ Command {command} ACK received with result: {msg.result}")
            return msg.result

def request_data_streams(master):
    """Request data streams to receive GLOBAL_POSITION_INT"""
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # Request position data
        10,  # Rate (Hz)
        1  # Start sending
    )
    # Request the data stream for NAV_CONTROLLER_OUTPUT
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1
    )
    print("📡 Requested data stream for position updates.")

def main():
    # master = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
    master = mavutil.mavlink_connection(**config)
    master.wait_heartbeat()
    print("✅ Heartbeat received.")

    master.target_system = 1
    master.target_component = 1
    print(f"✅ Fixed System ID: {master.target_system}, Fixed Component ID: {master.target_component}")

    # Request position data
    request_data_streams(master)

    # Set flight mode to GUIDED (needed for takeoff)
    mode_id = master.mode_mapping()["GUIDED"]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print("🚀 Switched to GUIDED mode.")

    time.sleep(2)

    # Arm the drone
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    wait_for_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print("✅ Drone armed.")

    # Automatic takeoff to 10m altitude
    target_altitude = 10
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude
    )
    wait_for_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    print(f"🚀 Taking off to {target_altitude}m...")

    # Wait until altitude is reached
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            altitude = msg.relative_alt / 1000.0  # Convert mm to meters
            print(f"📍 Current Altitude: {altitude}m")
            if altitude >= target_altitude * 0.95:  # 95% of target altitude
                print(f"✅ Reached {target_altitude}m altitude.")
                break
    
    prev = [
        [50, 50],
        [50, 0],
        [20, -50],
        [50, 0],
    ]

    # Wait until altitude is reached
    while True:
        msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        if msg:
            print(f"Distance to WP: {msg.wp_dist}m, Bearing: {msg.nav_bearing}°")
            if msg.wp_dist < 0.1:
                master.mav.set_position_target_local_ned_send(
                    0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111111000,
                    prev[0][0], prev[0][1], 0, 0, 0, 0, 0, 0, 0, 0, 0
                )
                prev.pop(0)

    print("🛑 Holding altitude, ready for further commands.")

if __name__ == '__main__':
    main()
