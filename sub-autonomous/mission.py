import json
import logging
import time
from pathlib import Path

from geopy.distance import geodesic

from config import (
    HOVER_DURATION_SECONDS,
    JSON_SAVE_INTERVAL_SECONDS,
    GPS_CLUSTER_THRESHOLD_M,
    TARGET_ALTITUDE,
    WAYPOINT_ACCEPTANCE_RADIUS,
    HOVER_ALTITUDE,
    FLYING_ALTITUDE
)
from controller import MavlinkController
from vision import Vision


logger = logging.getLogger(__name__)


class Mission:
    def __init__(self, controller, waypoints, vision = None):
        self.controller: MavlinkController = controller
        self.vision: Vision = vision

        # Waypoints are flown in this exact order.
        self.waypoints = waypoints

        self.visited = []
        self.gps_points = []
        self.last_json_save = 0.0
        self.gps_output_path = Path("gps_path.json")

    def _update_or_add_detection_point(self, lat, lon, alt, timestamp, pixel_dist, area):
        """Merge nearby detections so gps_path.json stores clustered points."""
        for index, point in enumerate(self.gps_points):
            distance_m = geodesic((lat, lon), (point["lat"], point["lon"])).meters
            if distance_m >= GPS_CLUSTER_THRESHOLD_M:
                continue

            if pixel_dist < point.get("pixel_dist", float("inf")):
                self.gps_points[index] = {
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "timestamp": timestamp,
                    "pixel_dist": pixel_dist,
                    "area": area,
                }
                return "UPDATED"

            return "DUPLICATE"

        self.gps_points.append(
            {
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "timestamp": timestamp,
                "pixel_dist": pixel_dist,
                "area": area,
            }
        )
        return "NEW"

    def _save_gps_path(self, force=False):
        now = time.time()
        if not force and (now - self.last_json_save) < JSON_SAVE_INTERVAL_SECONDS:
            return

        with self.gps_output_path.open("w", encoding="utf-8") as file:
            json.dump(self.gps_points, file, indent=4)

        self.last_json_save = now

    def _process_detection_and_log(self):
        detected, _, _, area, pixel_dist, display, mask = self.vision.detect_target()
        self.vision.show_preview(display, mask)

        if not detected:
            return

        lat, lon, alt = self.controller.get_gps_reading(alt=True)
        status = self._update_or_add_detection_point(
            lat=lat,
            lon=lon,
            alt=alt,
            timestamp=time.time(),
            pixel_dist=pixel_dist,
            area=area,
        )

        logger.info(
            "HSV detection %s | area=%.1f lat=%.7f lon=%.7f alt=%.2f",
            status,
            area,
            lat,
            lon,
            alt,
        )

        self._save_gps_path()

    def _wait_until_target_reached(self, target_lat, target_lon):
        while True:
            lat, lon = self.controller.get_gps_reading()
            dist_to_target = geodesic((lat, lon), (target_lat, target_lon)).meters

            if self.vision:
                self._process_detection_and_log()

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
            if self.vision:
                self._process_detection_and_log()
            time.sleep(0.1)

    def _return_to_launch_and_land(self, home_lat, home_lon):
        logger.info("Returning to launch")
        self.controller.set_mode("GUIDED")
        self.controller.send_navigate_command(home_lat, home_lon)

        self._wait_until_target_reached(home_lat, home_lon)

        self.controller.land()
        self.controller.disarm()

    def run_scout(self):
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
            self._process_detection_and_log()
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
        self._save_gps_path(force=True)
        self._return_to_launch_and_land(home_lat, home_lon)

    def run_sprayer(self):
        self.controller.set_max_velocity_params()
        self.controller.set_mode("GUIDED")
        self.controller.arm_and_takeoff(target_altitude = FLYING_ALTITUDE)

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