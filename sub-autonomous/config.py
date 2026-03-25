# ==========================================================
# SUB-AUTONOMOUS ROUND PARAMETERS (EDIT THESE)
# ==========================================================

TARGET_ALTITUDE = 2                 # meters
WAYPOINT_ACCEPTANCE_RADIUS = 2.5    # meters
HOVER_DURATION_SECONDS = 5          # seconds
ANGLE_DEG = 0                       # degrees (0 = north-south, 90 = east-west)
SPACING = 5.0

# ==========================================================
# HSV DETECTION + LOGGING PARAMETERS
# ==========================================================

CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# HSV range used in camera.py for yellow detection.
HSV_LOWER = [20, 70, 120]
HSV_UPPER = [35, 255, 255]

# Minimum contour area to treat as a valid detection.
AREA_THRESHOLD = 15000

# Distance threshold used to merge nearby GPS detections into one cluster.
GPS_CLUSTER_THRESHOLD_M = 2.0

# Save interval for gps_path.json writes.
JSON_SAVE_INTERVAL_SECONDS = 2.0

# Optional debug view for HSV mask.
SHOW_MASK_WINDOW = False