# ==========================================================
# SIMPLE WAYPOINT MISSION PARAMETERS
# ==========================================================

# MAVLink connection for SITL or flight controller.
MAVLINK_DEVICE = "tcp:127.0.0.1:5763"

FLYING_ALTITUDE = 2.5                # meters
HOVER_ALTITUDE = 1.5               # meters
WAYPOINT_ACCEPTANCE_RADIUS = 2.5     # meters
HOVER_DURATION_SECONDS = 5           # seconds

# Edit this list directly with [lat, lon] pairs.
WAYPOINTS = [
    [29.945022, 76.8172102],
	[29.944904, 76.8171532],
	[29.9447954, 76.8172652],
	[29.9448674, 76.8174248]
]
