import json
import logging
from geopy.distance import geodesic

from logger_config import setup_logging

setup_logging("mapping_gen_points")
logger = logging.getLogger(__name__)

# Load GPS data from JSON
with open("gps_paths.json", "r") as file:
    gps_data = json.load(file)  # Expecting a list of tuples [(lat, lon, detect), ...]

# Parameters
THRESHOLD = 15  # meters; adjust to define cluster radius
bullseye_locations = []
visited = set()

# Function to check if a point is near any existing cluster
def is_near_existing_clusters(lat, lon):
    for cluster_point in bullseye_locations:
        if geodesic((lat, lon), cluster_point).meters < THRESHOLD:
            return True
    return False

# Process GPS data
for lat, lon, detect in gps_data:
    if detect and (lat, lon) not in visited:
        if not is_near_existing_clusters(lat, lon):  
            bullseye_locations.append((lat, lon))  # Add only the first valid point of the cluster
        visited.add((lat, lon))

# Output result
logger.info("Detected Bullseye Locations: %s", bullseye_locations)
