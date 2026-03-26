# connection
SCOUT_CONNECTION = "tcp:127.0.0.1:5763"
SPRAYER_CONNECTION = "tcp:127.0.0.1:5764"

# kml
ANGLE_DEG = 0                       # degrees (0 = north-south, 90 = east-west)
SPACING = 2.0

# scout drone
TARGET_ALTITUDE = 2                 # meters
WAYPOINT_ACCEPTANCE_RADIUS = 2.5    # meters
HOVER_DURATION_SECONDS = 5          # seconds

# sprayer drone
FLYING_ALTITUDE = 2.5                # meters
HOVER_ALTITUDE = 1.5               # meters

# camera setup
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# HSV range used in camera.py for yellow detection.
HSV_LOWER = [24, 45, 149]
HSV_UPPER = [71, 255, 255]

# Minimum contour area to treat as a valid detection.
AREA_THRESHOLD = 8000

# Distance threshold used to merge nearby GPS detections into one cluster.
GPS_CLUSTER_THRESHOLD_M = 2.5

# Save interval for gps_path.json writes.
JSON_SAVE_INTERVAL_SECONDS = 2.0

# Optional debug view for HSV mask.
SHOW_MASK_WINDOW = False