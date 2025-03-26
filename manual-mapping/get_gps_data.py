import json
import sys
import socketio
import cv2
import numpy as np
from ultralytics import YOLO
from pymavlink import mavutil
from geopy.distance import geodesic

from picamera2 import Picamera2

print("Waiting for connection")

# Load config based on platform
config = json.load(open("../settings.json"))["rpi" if sys.platform == "linux" else "sim"]

master = mavutil.mavlink_connection(**config)
master.wait_heartbeat()
print("MAVLink Connection Established")

# Request GPS Data Stream
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

# Initialize SocketIO Client
sio = socketio.Client()
sio.connect('http://127.0.0.1:5000') 

model = YOLO("best-colab.pt", task = "detect").to("cpu")
print("Loaded yolo model")

# Open Camera
# cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(main={"size": (640, 480)}))
picam2.start()


def get_gps():
    """Fetch GPS data from MAVLink"""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7 
        lon = msg.lon / 1e7
        return {"lat": lat, "lon": lon}
    return None

def detect_bullseye():
    # ret, frame = cap.read()
    # if not ret:
    #     print("⚠️ Failed to capture image from camera.")
    #     return False

    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = model.predict(frame, imgsz=160, verbose = False, show = False)
    
    for result in results:
        for box in result.boxes:
            if box.conf > 0.5:  # Confidence threshold
                print("🎯 Bullseye detected!")
                return True
    
    print("❌ No bullseye detected.")
    return False


try:
    while True:
        gps_data = get_gps()
        bullseye_detected = detect_bullseye()
        if gps_data:
            gps_data["bullseye"] = bullseye_detected
            print(f"Sending Data: {gps_data}")
            if sio.connected:
                sio.emit('new_gps', gps_data)
        
except KeyboardInterrupt:
    print("Stopping GPS Stream")
    cap.release()
    sio.disconnect()
