import time, json
from pymavlink import mavutil
from geopy.distance import geodesic
import cv2
from ultralytics import YOLO
import math
import sys

from picamera2 import Picamera2

model = YOLO("best-colab.pt", task = "detect").to("cpu")
print("Loaded yolo model")
bullseye_locations = []
visited = set()
dev = False

picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (320, 320)}))
picam2.start()

config = json.load(open("../settings.json"))["rpi" if sys.platform == "linux" else "sim"]
master = mavutil.mavlink_connection(**config)

master.wait_heartbeat()
print("Heartbeat received, connected to drone.")

# default values are wrong sometimes
master.target_system = 1
master.target_component = 1

# gps data
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1
)

# waypoint data
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1
)

# autonomous unplanned missions work in guided mode
mode_id = master.mode_mapping()["GUIDED"]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("Switched to GUIDED mode.")

# need this because pymavlink is not blocking, and other libraries may instead block commands by pymavlink
def wait_for_ack(master, command):
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg and msg.command == command:
            print(f"✅ Command {command} ACK received with result: {msg.result}")
            return msg.result

def arm_and_takeoff(target_altitude=25):
    print("Arming drone and taking off...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    wait_for_ack(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, target_altitude
    )
    wait_for_ack(master, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

# nearest neighbour algorithm
def get_nearest_bullseye(current_lat, current_lon):
    min_distance = float('inf')
    nearest = None
    
    for lat, lon in bullseye_locations:
        if (lat, lon) not in visited:
            distance = geodesic((current_lat, current_lon), (lat, lon)).meters
            if distance < min_distance:
                min_distance = distance
                nearest = (lat, lon)
    
    return nearest

def send_navigate_command(lat, lon, alt=25):
    print(f"Navigating to: {lat}, {lon}")
    master.mav.mission_item_send(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0, lat, lon, alt
    )
    time.sleep(5)

def get_gps_reading(alt = False, heading_ = False):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    heading = msg.hdg / 100.0

    if alt:
        return lat, lon, msg.relative_alt / 1e3
    
    if heading_:
        return lat, lon, heading

    return lat, lon

def get_wp_reading():
    msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)

    return msg.wp_dist, msg.nav_bearing

# for simulation use only, to simulate the drone turning in a certain direction
def rotate_image(image, angle):
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)

    rotation_matrix = cv2.getRotationMatrix2D(center, -angle, 1.0)  # Negative to match coordinate system
    rotated_image = cv2.warpAffine(image, rotation_matrix, (w, h))

    return rotated_image

def detect_bullseye():
    # cap = cv2.VideoCapture(0)
    detected = False
    center_x, center_y = None, None
    
    for _ in range(10):
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # bottom mounted cam orientation
        if dev:
            frame = cv2.flip(frame, 1)
        else:
            frame = cv2.flip(frame, -1)

        # if not ret:
        #     continue

        # rotate frame to simulate the drone yaw rotation in bottom facing camera
        if dev:
            _, _, bearing = get_gps_reading(heading_ = True)
            angle_deg = (bearing + 360) % 360
            frame = rotate_image(frame, -angle_deg)
        
        results = model.predict(frame, imgsz = 320, verbose = False)
        for result in results:
            for box in result.boxes:
                if box.conf > 0.5:
                    detected = True
                    x_center = (box.xyxy[0][0] + box.xyxy[0][2]) / 2
                    y_center = (box.xyxy[0][1] + box.xyxy[0][3]) / 2
                    center_x, center_y = x_center, y_center
                    
                    # show preview if in dev mode
                    if dev:
                        x1, y1, x2, y2 = box.xyxy[0]

                        # drone frame
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                      color=(0, 255, 0), thickness=2)
                        
                        # confidence label
                        label = f"{round(float(box.conf), 3)}"
                        cv2.putText(frame, label, (int(x1), int(y1) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # center point
                        cv2.circle(frame, (int(x_center), int(y_center)), 5, (255, 0, 0), -1)
                        cv2.circle(frame, (frame.shape[1] // 2, frame.shape[0] // 2),
                                   5, (255, 0, 0), -1)
                        
                        # tolerance frame
                        cv2.rectangle(frame, (frame.shape[1] // 2 - 60, frame.shape[0] // 2 - 60),
                                      (frame.shape[1] // 2 + 60, frame.shape[0] // 2 + 60),
                                      color=(0, 0, 255), thickness=2)

                        cv2.imshow("YOLO Results", frame)
                        cv2.waitKey(1)
                    
                    break
            if detected:
                break
        if detected:
            break
    
    return detected, center_x, center_y

def move_relative(distance, bearing_deg):
    bearing_deg = max(-10, min(10, bearing_deg))

    bearing_rad = math.radians(bearing_deg)
    
    position_x = distance * math.cos(bearing_rad)  # Forward
    position_y = distance * math.sin(bearing_rad)  # Right

    print(f"Moving: {distance}m at {bearing_deg}° (X={position_x:.2f}, Y={position_y:.2f})")

    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111111000,  # Disable yaw, yaw rate, and altitude control
        position_x, position_y, 0,  # Target X, Y positions, Z set to 0 (ignored due to mask)
        0, 0, 0,  # Velocity (ignored)
        0, 0, 0,  # Acceleration (ignored)
        0, 0  # Yaw, Yaw rate (ignored due to mask)
    )

def adjust_drone_position(center_x, center_y, frame_width=640, frame_height=480):
    tolerance = 60
    dx = center_x - (frame_width / 2)
    dy = center_y - (frame_height / 2)
    
    if abs(dx) > tolerance or abs(dy) > tolerance:
        angle_rad = math.atan2(dy, dx)  # atan2 automatically handles quadrants
        angle_deg = math.degrees(angle_rad)

        bearing = (angle_deg + 360) % 360 + 90

        # if bearing > 180, make it negative
        if bearing > 180:
            bearing = bearing - 360

        move_relative(2, bearing)

        print("Required shift:", bearing)

        return False
    return True

def hover_and_record():
    readings = []
    print("Hovering and recording GPS data...")
    
    for _ in range(5):
        readings.append(get_gps_reading())
        time.sleep(1)
    
    avg_lat = sum(r[0] for r in readings) / len(readings)
    avg_lon = sum(r[1] for r in readings) / len(readings)
    return avg_lat, avg_lon

arm_and_takeoff()

# wait for altitude to reach 25m
_,_,alt = get_gps_reading(alt = True)
while alt < 24:
    _,_,alt = get_gps_reading(alt = True)
    print(f"Altitude: {alt}")

while len(visited) < len(bullseye_locations):
    current_lat, current_lon = get_gps_reading()
    target = get_nearest_bullseye(current_lat, current_lon)
    
    if target:
        total_distance = geodesic((current_lat, current_lon), target).meters  # Total distance to target
        traveled_distance = 0  # Track distance traveled
        
        send_navigate_command(*target)
        
        print("Moving towards next bullseye...")

        while traveled_distance < 0.7 * total_distance:
            new_lat, new_lon = get_gps_reading()
            traveled_distance = geodesic((current_lat, current_lon), (new_lat, new_lon)).meters
            print(f"Traveled: {traveled_distance:.2f}m / {total_distance:.2f}m")

        print("Detection enabled!")

        centered = False
        while not centered:
            detected, center_x, center_y = detect_bullseye()
            if detected:
                centered = adjust_drone_position(center_x, center_y)

                req_dist, _ = get_wp_reading()
                while req_dist > 0.15:
                    req_dist, _ = get_wp_reading()
                    print("Waiting for precision...")

            else:
                print("Bullseye not detected, going back to target location...")
                send_navigate_command(*target)


        print("Bullseye detected, hovering and recording.")
        avg_lat, avg_lon = hover_and_record()
        visited.add(target)

        print(f"Bullseye confirmed at: {avg_lat}, {avg_lon}")

        with open("visited.json", "w") as f2:
            json.dump(list(visited), f2, indent=4)
    else:
        print("No more targets remaining.")
        break

print("Mission complete.")