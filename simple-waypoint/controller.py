import logging
import time

from pymavlink import mavutil

from config import MAVLINK_DEVICE, FLYING_ALTITUDE

logger = logging.getLogger(__name__)

class MavlinkController:
    def __init__(self):
        self.master = mavutil.mavlink_connection(MAVLINK_DEVICE)
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
            1,
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            2,
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

    def arm_and_takeoff(self, target_altitude=FLYING_ALTITUDE):
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

    def get_gps_reading(self, alt=False):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

        lat = msg.lat / 1e7
        lon = msg.lon / 1e7

        if alt:
            return lat, lon, msg.relative_alt / 1e3

        return lat, lon

    def send_navigate_command(self, lat, lon, alt=FLYING_ALTITUDE):
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
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            if msg and msg.relative_alt <= 0.4:
                logger.info("Landed successfully")
                break
            else:
                logger.info("Current altitude during landing: %.2fm", msg.relative_alt / 1e3)
            time.sleep(0.1)

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

    def set_max_velocity_params(self):
        for param in [b"WPNAV_SPEED", b"WPNAV_SPEED_UP", b"WPNAV_SPEED_DN"]:
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param,
                75,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
