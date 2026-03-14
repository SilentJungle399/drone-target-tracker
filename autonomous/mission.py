import time
import json
import logging

from geopy.distance import geodesic

from config import (
    TARGET_ALTITUDE,
    WAYPOINT_ACCEPTANCE_RADIUS,
    CENTER_TOLERANCE,
    CORRECTION_SPEED,
    FRAME_WIDTH,
    FRAME_HEIGHT,
)

from controller import MavlinkController
from vision import Vision


logger = logging.getLogger(__name__)


class Mission:
    def __init__(self, controller, vision):
        self.controller: MavlinkController = controller
        self.vision: Vision = vision
        self.bullseye_locations = [
            [29.9499810, 76.8160079],
            [29.9498973, 76.8164974],
            [29.9493210, 76.8165068]

            # [29.9471718, 76.8166544],
            # [29.9471915, 76.8165109],
            # [29.9472107, 76.8163707]

            # [29.9448095, 76.8174168],
            # [29.9449071, 76.8172728],
            # [29.94508, 76.8172765]
        ]
        self.visited = set()

    def get_nearest_bullseye(self, current_lat, current_lon):
        min_distance = float("inf")
        nearest = None

        # for lat, lon in self.bullseye_locations:
        #     if (lat, lon) not in self.visited:
        #         dist = geodesic(
        #             (current_lat, current_lon),
        #             (lat, lon)
        #         ).meters

        #         if dist < min_distance:
        #             min_distance = dist
        #             nearest = (lat, lon)

        # return first unvisited target (no dynamic nearest selection)
        for lat, lon in self.bullseye_locations:
            if (lat, lon) not in self.visited:
                nearest = (lat, lon)
                break

        return nearest

    def hover_and_record(self):
        readings = []

        logger.info("Recording points:")
        logger.info("-----------------")
        # for i in range(5):
        #     coords = self.controller.get_gps_reading()
        #     logger.info("%s.) %s", i + 1, coords)
        #     readings.append(coords)
        #     break
        #     time.sleep(1)

        t1 = time.time()

        while time.time() - t1 < 11:
            coords = self.controller.get_gps_reading()
            
            if len(readings) % 10 == 0:
                logger.info("%s.) %s", len(readings) + 1, coords)

            readings.append(coords)
            time.sleep(0.1)

        lat = sum(r[0] for r in readings) / len(readings)
        lon = sum(r[1] for r in readings) / len(readings)

        logger.info("Final coords: %s, %s", lat, lon)

        return lat, lon

    def return_land_disarm(self, home_lat, home_lon):
        logger.info("Returning to home")

        self.controller.set_mode("GUIDED")
        self.controller.send_navigate_command(home_lat, home_lon)

        start_lat, start_lon = self.controller.get_gps_reading()
        total_dist = geodesic((start_lat, start_lon), (home_lat, home_lon)).meters

        last_print = 0

        while True:
            lat, lon = self.controller.get_gps_reading()
            dist_to_home = geodesic((lat, lon), (home_lat, home_lon)).meters

            progress = 1 - (dist_to_home / total_dist)

            if progress - last_print >= 1:
                logger.info("%s%% complete | %.2fm remaining", int(progress * 100), dist_to_home)
                last_print = progress

            if dist_to_home < 2:  # within 2 meters of home
                logger.info("Reached home")
                break

            self.vision.show_preview()
            # time.sleep(1)

        self.controller.land()
        self.controller.disarm()

    def adjust_drone_position(self, center_x, center_y):
        dx = center_x - FRAME_WIDTH / 2
        dy = center_y - FRAME_HEIGHT / 2

        if abs(dx) < CENTER_TOLERANCE and abs(dy) < CENTER_TOLERANCE:
            self.controller.send_ned_velocity(0, 0, 0)
            return True

        # NED frame: x=North, y=East
        # dx>0 (bullseye right)  -> move East  -> vy positive
        # dy>0 (bullseye below)  -> move South -> vx negative
        vx = 0.0
        vy = 0.0

        if abs(dy) > CENTER_TOLERANCE:
            vx = -CORRECTION_SPEED if dy > 0 else CORRECTION_SPEED

        if abs(dx) > CENTER_TOLERANCE:
            vy = CORRECTION_SPEED if dx > 0 else -CORRECTION_SPEED

        logger.info("NED correction -> vx %s vy %s", vx, vy)
        self.controller.send_ned_velocity(vx, vy, 0)

        return False

    def run(self):
        # set params
        self.controller.set_max_velocity_params()

        self.controller.set_mode("GUIDED")
        self.controller.arm_and_takeoff()

        home_lat, home_lon = self.controller.get_gps_reading()
        logger.info("Home position saved: %s %s", home_lat, home_lon)

        _, _, alt = self.controller.get_gps_reading(alt=True)
        start = time.time()

        while alt < TARGET_ALTITUDE * 0.9:
            if time.time() - start > 20:
                raise Exception("Takeoff timeout")

            _, _, alt = self.controller.get_gps_reading(alt=True)
            logger.info("Altitude: %s", alt)
            self.vision.show_preview()

        while len(self.visited) < len(self.bullseye_locations):
            current_lat, current_lon = self.controller.get_gps_reading()
            target = self.get_nearest_bullseye(current_lat, current_lon)

            if not target:
                break

            self.controller.send_navigate_command(*target)
            logger.info("Flying to target")

            req_dist = geodesic((current_lat, current_lon), target).meters
            travelled_distance = 0

            logger.info("From %s to %s", (current_lat, current_lon), target)
            logger.info("Required distance: %s", req_dist)

            while True:
                new_lat, new_lon = self.controller.get_gps_reading()
                dist_to_target = geodesic((new_lat, new_lon), target).meters

                if dist_to_target < 2:  # arrival threshold
                    break

                logger.info("Distance remaining: %s", dist_to_target)
                self.vision.show_preview()

            logger.info("Arrived at waypoint")

            corrections = 0
            centered = False
            no_detection_since = None
            lost = False                       # to avoid spamming in console

            while True:
                detected, center_x, center_y, display_frame = self.vision.detect_bullseye()
                self.vision.show_preview(display_frame)

                if not detected:
                    if not lost:
                        logger.info("Target lost")

                    lost = True
                    no_detection_since = time.time() if no_detection_since is None else no_detection_since

                    if no_detection_since and time.time() - no_detection_since > 5:
                        logger.info("No detection for 5 seconds, moving to next target")
                        break

                    self.controller.send_ned_velocity(0, 0, 0)
                    time.sleep(0.05)
                    continue
                else:
                    lost = False
                    no_detection_since = None

                centered = self.adjust_drone_position(center_x, center_y)

                if centered:
                    break

                corrections += 1
                time.sleep(0.05)

            if centered:
                logger.info("Bullseye centered")
            else:
                logger.info("Max corrections reached")

            lat, lon = self.hover_and_record()
            self.visited.add(target)
            logger.info("Recorded: %s %s", lat, lon)

            with open("visited.json", "w") as f:
                json.dump(list(self.visited), f, indent=4)

            self.controller.set_mode("GUIDED")

        logger.info("Mission complete")
        self.return_land_disarm(home_lat, home_lon)
