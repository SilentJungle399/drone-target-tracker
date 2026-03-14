import json
import os
import sys
import logging
import cv2
import numpy as np
from ultralytics import YOLO
from pymavlink import mavutil
from geopy.distance import geodesic
import time

from logger_config import setup_logging

setup_logging("mapping")
logger = logging.getLogger(__name__)

logger.info("Waiting for connection")

master = mavutil.mavlink_connection("tcp:127.0.0.1:5763")

gps_points = []

try:
    master.wait_heartbeat(timeout=30)
    logger.info("MAVLink Connection Established")
except Exception as e:
    logger.exception("MAVLink Connection Failed: %s", e)
    logger.info("Continuing in simulation/debug mode...")
    master = None

# Request GPS Data Stream if connected
if master:
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        1,
        1
    )

try:
    model = YOLO("best-colab.pt", task="detect").to("cuda")
    logger.info("Loaded YOLO model")
except Exception as e:
    logger.exception("Failed to load YOLO model: %s", e)
    logger.info("Make sure best-colab.pt exists in the current directory")
    sys.exit(1)

logger.info("Initializing webcam...")
cap = cv2.VideoCapture(0)

logger.info("Starting main loop...")

# Last known valid GPS state
last_lat = None
last_lon = None
last_alt = None

# JSON save throttle
last_save_time = 0

# FPS limiter
frame_interval = 1 / 20
last_frame_time = 0


def update_or_add_point(lat, lon, alt, timestamp, pixel_dist, threshold=2):
    """Add a new GPS point, or overwrite an existing nearby one if this detection is closer to the frame centre.
    Returns 'NEW', 'UPDATED', or 'DUPLICATE'."""
    for i, pt in enumerate(gps_points):
        if geodesic((lat, lon), (pt["lat"], pt["lon"])).meters < threshold:
            if pixel_dist < pt.get("pixel_dist", float("inf")):
                gps_points[i] = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "timestamp": timestamp,
                    "pixel_dist": pixel_dist,
                }
                return "UPDATED"
            return "DUPLICATE"
    gps_points.append({
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "timestamp": timestamp,
        "pixel_dist": pixel_dist,
    })
    return "NEW"


# Main loop
try:
    while True:
        now = time.time()

        # Capture frame from webcam
        ret, frame = cap.read()
        if not ret:
            logger.error("Failed to grab frame")
            break

        frame = cv2.flip(frame, -1)

        # --- FPS limiter: skip heavy processing until interval has passed ---
        if now - last_frame_time < frame_interval:
            # Still show the last annotated frame to keep display smooth
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            continue

        last_frame_time = now

        # --- Update last known GPS state ---
        if master:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                last_lat = msg.lat / 1e7
                last_lon = msg.lon / 1e7
                last_alt = msg.alt / 1000.0
                logger.info("Lat=%.6f, Lon=%.6f | Points: %s", last_lat, last_lon, len(gps_points))

        # --- Run YOLO detection ---
        results = model(frame, verbose=False)
        annotated_frame = results[0].plot()

        h, w = annotated_frame.shape[:2]

        # --- Draw acceptance region (centre 50% of frame) ---
        ax1, ay1 = int(0.40 * w), int(0.40 * h)
        ax2, ay2 = int(0.60 * w), int(0.60 * h)
        cv2.rectangle(annotated_frame, (ax1, ay1), (ax2, ay2), (0, 255, 255), 2)

        # --- Process detections ---
        for r in results:
            for box in r.boxes:
                confidence = float(box.conf[0])
                if confidence <= 0.7:
                    continue

                class_name = model.names[int(box.cls[0])]
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                x = lambda prefix: logger.info("[%s] Detected: conf=%.2f center=(%.0f,%.0f)", prefix, confidence, cx, cy)

                # Only record if centre is inside acceptance region
                if not (ax1 <= cx <= ax2 and ay1 <= cy <= ay2):
                    x("OUTSIDE")
                    continue

                if last_lat is None or last_lon is None:
                    x("NO GPS")
                    continue

                frame_cx, frame_cy = w / 2, h / 2
                pixel_dist = ((cx - frame_cx) ** 2 + (cy - frame_cy) ** 2) ** 0.5

                result = update_or_add_point(last_lat, last_lon, last_alt, now, pixel_dist)
                x(result)

                if result in ("NEW", "UPDATED"):
                    logger.info("====================")
                    logger.info(
                        "GPS Point %s: lat=%s, lon=%s, alt=%s, pixel_dist=%.1f",
                        result,
                        last_lat,
                        last_lon,
                        last_alt,
                        pixel_dist,
                    )
                    logger.info("====================")


        # --- Throttled JSON save (every 2 seconds) ---
        if time.time() - last_save_time > 2:
            json.dump(gps_points, open("gps_path.json", "w"), indent=4)
            last_save_time = time.time()

        # --- Add connection status overlay ---
        if master:
            gps_status = f"GPS: ({last_lat:.5f}, {last_lon:.5f})" if last_lat is not None else "GPS: waiting..."
            cv2.putText(annotated_frame, gps_status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(annotated_frame, "Simulation Mode", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # --- Resize for display if needed ---
        if w > 1024:
            scale = 1024 / w
            annotated_frame = cv2.resize(annotated_frame,
                                         (int(w * scale), int(h * scale)))

        cv2.imshow("YOLO Detection (VS Code)", annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

except KeyboardInterrupt:
    logger.info("Interrupted by user")
except Exception as e:
    raise e
finally:
    # Final save before exit
    json.dump(gps_points, open("gps_path.json", "w"), indent=4)
    logger.info("Cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    logger.info("Done")