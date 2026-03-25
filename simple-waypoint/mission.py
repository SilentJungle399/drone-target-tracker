import json
import logging
import time

from geopy.distance import geodesic

from config import HOVER_DURATION_SECONDS, FLYING_ALTITUDE, HOVER_ALTITUDE, WAYPOINT_ACCEPTANCE_RADIUS
from controller import MavlinkController

logger = logging.getLogger(__name__)

class Mission:
    def __init__(self, controller, waypoints):
        self.controller: MavlinkController = controller
        self.waypoints = [tuple(wp) for wp in waypoints]
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
        logger.info("Returning to home")
        self.controller.set_mode("GUIDED")
        self.controller.send_navigate_command(home_lat, home_lon)

        self._wait_until_target_reached(home_lat, home_lon)

        self.controller.land()
        self.controller.disarm()

    def run(self):
        if not self.waypoints:
            raise ValueError("No waypoints provided. Add points in simple-waypoint/config.py")

        self.controller.set_max_velocity_params()
        self.controller.set_mode("GUIDED")
        self.controller.arm_and_takeoff()

        home_lat, home_lon = self.controller.get_gps_reading()
        logger.info("Home position saved: %.7f, %.7f", home_lat, home_lon)

        start = time.time()
        while True:
            _, _, altitude = self.controller.get_gps_reading(alt=True)

            if altitude >= FLYING_ALTITUDE * 0.9:
                break

            if time.time() - start > 20:
                raise TimeoutError("Takeoff timeout")

            logger.info("Altitude: %.2fm", altitude)
            time.sleep(0.05)

        for target_lat, target_lon in self.waypoints:
            logger.info("Navigating to waypoint: %.7f, %.7f", target_lat, target_lon)
            self.controller.send_navigate_command(target_lat, target_lon)
            self._wait_until_target_reached(target_lat, target_lon)

            self.controller.change_altitude(HOVER_ALTITUDE)
            self._hover_for_duration(HOVER_DURATION_SECONDS)
            self.controller.change_altitude(FLYING_ALTITUDE)
            
            self.visited.append([target_lat, target_lon])

            with open("visited_simple_waypoint.json", "w", encoding="utf-8") as file:
                json.dump(self.visited, file, indent=4)

        logger.info("Waypoints complete")
        self._return_to_launch_and_land(home_lat, home_lon)
