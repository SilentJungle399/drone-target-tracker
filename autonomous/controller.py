import time
import json
import sys
import math
import logging

from pymavlink import mavutil

from config import TARGET_ALTITUDE


logger = logging.getLogger(__name__)


class MavlinkController:

    def __init__(self):
        config = {
            "device": "tcp:127.0.0.1:5763",
        }

        self.master = mavutil.mavlink_connection(**config)
        self.master.wait_heartbeat()
        logger.info("Heartbeat received")

        self.master.target_system = 1
        self.master.target_component = 1

        self._request_streams()

    def _request_streams(self):
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            5,
            1
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            2,
            1
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # includes ATTITUDE
            10,
            1
        )

    def wait_for_ack(self, command):
        while True:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)

            if msg and msg.command == command:
                logger.info("ACK received %s", msg.result)
                return msg.result

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
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
            1, 0, 0, 0, 0, 0, 0
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        logger.info("Takeoff")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def get_gps_reading(self, alt=False, heading_=False):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        heading = msg.hdg / 100.0

        if alt:
            return lat, lon, msg.relative_alt / 1e3

        if heading_:
            return lat, lon, heading

        return lat, lon

    def get_wp_reading(self):
        msg = self.master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)

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
            0, 0, 0, 0,
            lat,
            lon,
            alt
        )

    def _get_current_yaw(self):
        """Returns current yaw in radians from ATTITUDE telemetry.
        Falls back to GLOBAL_POSITION_INT heading if ATTITUDE is unavailable."""
        att = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if att:
            return att.yaw  # already in radians, NED frame
        # Fallback: convert compass heading (cdeg) from GLOBAL_POSITION_INT to radians
        gps = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        if gps:
            return math.radians(gps.hdg / 100.0)
        return 0.0

    def send_ned_velocity(self, vx, vy, vz=0.0):
        # type_mask: ignore position + acceleration + yaw_rate; use velocity + yaw
        # bit=1 means IGNORE, bit=0 means USE
        # bits: yaw_rate(11) yaw(10) az(8) ay(7) ax(6) | vz(5) vy(4) vx(3) | z(2) y(1) x(0)
        # bit 10 cleared → yaw IS used (drone holds current heading)
        # bit 11 set     → yaw_rate is ignored
        type_mask = 0b111111000111  # 2503

        yaw = self._get_current_yaw()

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,       # position (ignored)
            vy, (-1)*vx, vz,    # velocity m/s
            # vx, vy, vz,    # velocity m/s
            # (-1)*vx, (-1)*vy, (-1)*vz,    # velocity m/s
            0, 0, 0,       # acceleration (ignored)
            0, 0         # yaw locked to current heading; yaw_rate ignored
        )
            
    def land(self):
        logger.info("Landing")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_LAND)

    def disarm(self):
        logger.info("Disarming")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

        self.wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    def set_max_velocity_params(self):
        for param in [
            b'WPNAV_SPEED',
            b'WPNAV_SPEED_UP',
            b'WPNAV_SPEED_DN'
        ]:
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param,
                75,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
