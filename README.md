# Autonomous Bullseye Detection and Mapping

This repository documents the entire development process, from initial sensor tests to full autonomous flight implementation.

## Project Overview
This project involves mapping a field through manual flight to identify possible bullseye locations. The drone then executes an autonomous flight, taking off to 25m and navigating through waypoints while searching for a bullseye using a downward-facing camera. An object detection model ([SilentJungle399/bullseye-model](https://github.com/SilentJungle399/bullseye-model)) is used to detect the bullseye. Upon detecting a bullseye, the drone centers itself over it, hovers for 5 seconds to record GPS coordinates, and proceeds to the next waypoint. This process continues until all waypoints are cleared.

## Approach
The project began with initial testing to verify Pixhawk connectivity, retrieve sensor and GPS data, and send MAVLink commands.

To avoid real-world risks, Mission Planner SITL (Software in the Loop) was used for simulation. Attempts to stabilize flight using raw PWM inputs failed in SITL, so waypoint-based guided flight was used instead, proving successful for navigation.

For manual mapping, the drone was piloted to log GPS tracks and detect bullseyes. A Folium-based webserver enabled real-time visualization, helping map bullseye locations and transition to the autonomous phase.

In autonomous flight, waypoint navigation was combined with bullseye detection. When mounted on a real drone, the bottom-facing camera naturally rotates with yaw movements. However, in SITL, this motion had to be manually simulated to ensure accurate bullseye detection. To improve GPS accuracy, the drone hovered over detected bullseyes for 5 seconds, averaging multiple readings before recording the final location.

## Project Structure

### `mavlink-test/`
Contains initial tests to verify Pixhawk connectivity and MAVLink command execution.
- `get_data.py` - Tests retrieval of pitch/roll data from Pixhawk.
- `gps_test.py` - Fetches longitude/latitude data.
- `arm_test.py` - Tests arming the drone via MAVLink.
- `raw_input.py` - Attempts stable flight using raw PWM inputs (failed in SITL).
- `waypoint.py` - Tests guided flight using waypoints (successful).
- `img_test.py` - Simulates drone yaw motion effect on the camera (used only in SITL).

### `manual-mapping/`
Handles manual flight data logging and real-time visualization.
- `map.py` - Webserver (Flask + Socket.io) for live GPS tracking; saves data to `gps_path.json` and publishes a map (powered by Folium)
- `get_gps_data.py` - Fetches GPS data from Pixhawk and runs the AI model to detect bullseyes, sending updates to `map.py` via Socket.io.
- `gen_points.py` - Cleans `gps_path.json` and extracts waypoints for autonomous flight.

### `autonomous-flight/`
Handles the full autonomous flight execution.
- `visited.json` - Stores GPS coordinates of detected bullseyes.
- `main.py` - Implements the autonomous flight logic (majority of the code is commented).

## Notes
- The object detection model must be set up separately.
- Ensure correct MAVLink connection settings before running the scripts.
- Autonomous flight logic is still in testing; some parts are commented out for debugging.
