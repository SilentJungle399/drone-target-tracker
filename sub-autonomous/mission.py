import json
import logging
import time

from geopy.distance import geodesic

from config import TARGET_ALTITUDE, WAYPOINT_ACCEPTANCE_RADIUS, HOVER_DURATION_SECONDS
from controller import MavlinkController


logger = logging.getLogger(__name__)


class Mission:
    def __init__(self, controller):
        self.controller: MavlinkController = controller

        # Waypoints are flown in this exact order.
        self.waypoints = [
            [29.8677,	77.89946],
			# [29.86793,	77.89948],
			# [29.86803,	77.89935],
			# [29.86804,	77.89912],
			# [29.86779,	77.8991],
			# [29.86753,	77.89905],
			# [29.86739,	77.89908],
			# [29.86733,	77.89925],
			# [29.86745,	77.89936],
			[29.86762,	77.89932],
        ]

        self.visited = []

    def _wait_until_target_reached(self, target_lat, target_lon):
        while True:
            lat, lon = self.controller.get_gps_reading()
            dist_to_target = geodesic((lat, lon), (target_lat, target_lon)).meters

            if dist_to_target <= WAYPOINT_ACCEPTANCE_RADIUS:
                logger.info("Reached waypoint (%.2fm)", dist_to_target)
                return

            logger.info("Distance remaining: %.2fm", dist_to_target)
            time.sleep(0.1)

    def _hover_for_duration(self, seconds):
        logger.info("Hovering for %ss", seconds)
        end_time = time.time() + seconds

        while time.time() < end_time:
            lat, lon = self.controller.get_gps_reading()
            logger.info("Hover position: %.7f, %.7f", lat, lon)
            time.sleep(0.1)

    def _return_to_launch_and_land(self, home_lat, home_lon):
        logger.info("Returning to launch")
        self.controller.set_mode("GUIDED")
        self.controller.send_navigate_command(home_lat, home_lon)

        self._wait_until_target_reached(home_lat, home_lon)

        self.controller.land()
        self.controller.disarm()

    def run(self):
        self.controller.set_max_velocity_params()
        self.controller.set_mode("GUIDED")
        self.controller.arm_and_takeoff()

        home_lat, home_lon = self.controller.get_gps_reading()
        logger.info("Home position saved: %.7f, %.7f", home_lat, home_lon)

        start = time.time()
        while True:
            _, _, altitude = self.controller.get_gps_reading(alt=True)

            if altitude >= TARGET_ALTITUDE * 0.9:
                break

            if time.time() - start > 20:
                raise TimeoutError("Takeoff timeout")

            logger.info("Altitude: %.2fm", altitude)
            time.sleep(0.05)

        for target_lat, target_lon in self.waypoints:
            logger.info("Navigating to waypoint: %.7f, %.7f", target_lat, target_lon)
            self.controller.send_navigate_command(target_lat, target_lon)
            self._wait_until_target_reached(target_lat, target_lon)

            self._hover_for_duration(HOVER_DURATION_SECONDS)
            self.visited.append([target_lat, target_lon])

            with open("visited_sub_autonomous.json", "w", encoding="utf-8") as file:
                json.dump(self.visited, file, indent=4)

        logger.info("Waypoints complete")
        self._return_to_launch_and_land(home_lat, home_lon)
