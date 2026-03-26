import logging
import math
import time

from pymavlink import mavutil

from config import TARGET_ALTITUDE


logger = logging.getLogger(__name__)


class MavlinkController:

    def __init__(self, device):
        config = {
            "device": device,
        }

        self.master = mavutil.mavlink_connection(**config)
        self.master.wait_heartbeat()
        logger.info("Heartbeat received")

        self.master.target_system = 1
        self.master.target_component = 1

        self._request_streams()
        self.set_max_velocity_params()

    def destroy(self):
        self.master.close()

    def _request_streams(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            5,
            1,
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            2,
            1,
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,
            1,
        )

    def wait_for_ack(self, command):
        while True:
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True)

            if msg and msg.command == command:
                logger.info("ACK received %s", msg.result)
                return msg.result

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        logger.info("Mode -> %s", mode)
        time.sleep(2)

    def arm_and_takeoff(self, target_altitude=TARGET_ALTITUDE):
        logger.info("Arming")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        logger.info("Takeoff")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            target_altitude,
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def get_gps_reading(self, alt=False, heading_=False):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        heading = msg.hdg / 100.0

        if alt:
            return lat, lon, msg.relative_alt / 1e3

        if heading_:
            return lat, lon, heading

        return lat, lon

    def get_wp_reading(self):
        msg = self.master.recv_match(type="NAV_CONTROLLER_OUTPUT", blocking=True)

        return msg.wp_dist, msg.nav_bearing

    def send_navigate_command(self, lat, lon, alt=TARGET_ALTITUDE):
        logger.info("Navigating -> %s %s", lat, lon)

        self.master.mav.mission_item_send(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,
            0,
            0,
            0,
            0,
            0,
            lat,
            lon,
            alt,
        )

    def _get_current_yaw(self):
        """Returns current yaw in radians from ATTITUDE telemetry.
        Falls back to GLOBAL_POSITION_INT heading if ATTITUDE is unavailable."""
        att = self.master.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
        if att:
            return att.yaw

        gps = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if gps:
            return math.radians(gps.hdg / 100.0)

        return 0.0

    def send_ned_velocity(self, vx, vy, vz=0.0):
        type_mask = 0b111111000111

        yaw = self._get_current_yaw()

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0,
            0,
            0,
            vy,
            (-1) * vx,
            vz,
            0,
            0,
            0,
            yaw,
            0,
        )

    def land(self):
        logger.info("Landing")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)
        
        while True:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
            if msg and msg.relative_alt / 100 <= 0.4:
                logger.info("Landed successfully")
                break
            elif msg:
                logger.info("Current altitude during landing: %.2fm", msg.relative_alt / 1e3)
                print(msg.relative_alt / 100, 0.4)

    def change_altitude(self, alt):
        logger.info("Changing altitude -> %s", alt)

        lat, lon, current_alt = self.get_gps_reading(alt = True)

        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b110111111000),  # ignore everything except position
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        while True:
            _, _, current_alt = self.get_gps_reading(alt=True)
            logger.info("Current altitude: %.2f", current_alt)

            if abs(current_alt - alt) <= 0.5:
                logger.info("Target altitude reached")
                break

    def disarm(self):
        logger.info("Disarming")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    def set_max_velocity_params(self):
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b"",
            0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

        for param in [b"WPNAV_SPEED", b"WPNAV_SPEED_UP", b"WPNAV_SPEED_DN"]:
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param,
                75,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
