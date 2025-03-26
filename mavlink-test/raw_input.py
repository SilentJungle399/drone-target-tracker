import json, time, sys
from pymavlink import mavutil

config = json.load(open("settings.json"))["rpi" if sys.platform == "linux" else "sim"]

master = mavutil.mavlink_connection(**config)
master.wait_heartbeat()
print("received heartbeat")

master.target_system = 1
master.target_component = 1
print(f"✅ Fixed System ID: {master.target_system}, Fixed Component ID: {master.target_component}")

master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
print("requested height data")

def send_input_until(_time, x, y, z, r ):
    start_time = time.time()
    while time.time() - start_time < _time:
        print(f"Sending input: {x}, {y}, {z}, {r}")
        master.mav.manual_control_send(master.target_system, x, y, z, r, 0)
        time.sleep(0.1)

def send_input(x, y, z, r):
    """Sends manual control input to the drone."""
    master.mav.manual_control_send(master.target_system, x, y, z, r, 0)

def achieve_height(target_height):
    """
    Uses a PID controller to smoothly adjust throttle and achieve the target altitude.
    """
    Kp = 4  # Proportional gain (increase if response is too slow)
    Ki = 0.007   # Integral gain (helps reduce steady-state error)
    Kd = 15  # Derivative gain (dampens oscillations)

    integral = 0
    last_error = 0

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            altitude = msg.relative_alt / 1000.0  # Convert from mm to meters
            error = target_height - altitude
            integral += error
            derivative = error - last_error

            # PID output (adjusts throttle value)
            throttle = int(500  + (Kp * error) + (Ki * integral) + (Kd * derivative))
            throttle_inp = max(300, min(800, throttle))  # Constrain throttle between 30% and 80%
            # instead of hard limit, make it sloped
            # throttle_inp = int(300 + (throttle - 500) * 0.2)

            print(f"📍 Target: {target_height}m | Error: {error:.2f} | Integral: {integral:.2f} | Derivative: {derivative:.2f} | Altitude: {altitude:.2f}m | (500 + ({round(Kp * error, 2)}) + ({round(Ki * integral, 2)}) + ({round(Kd * derivative, 2)})) | Throttle: {throttle}->{throttle_inp} ")

            send_input(0, 0, throttle_inp, 0)

            # Exit loop when close to the target height
            # if abs(error) < 0.2:  # ±20 cm tolerance
            #     print(f"✅ Reached target height: {target_height}m")
                # break

            last_error = error
            time.sleep(0.1)

mode_id = master.mode_mapping()["STABILIZE"]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("STABILIZE mode set.")

master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print("arming...")
        break

# send_input_until(3, 0, 0, 500, 0, 0)
# print("sent throttle up 20%")

# send_input_until(5, 0, 0, 700, 0, 0)
# print("sent throttle up 50%")

# send_input_until(20, 0, 0, 400, 0, 0)

# Achieve target altitude
target_altitude = 10  # Set target altitude (meters)
achieve_height(target_altitude)

print("🛑 Holding position.")
send_input_until(10, 0, 0, 500, 0)  # Maintain neutral throttle

print("releasing throttle")
send_input_until(1, 0, 0, 0, 0)

print("released throttle and sticks")

master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print("disarmed")
        break