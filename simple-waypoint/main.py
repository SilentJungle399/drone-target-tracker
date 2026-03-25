from logger_config import setup_logging

setup_logging("simple_waypoint")

from config import WAYPOINTS
from controller import MavlinkController
from mission import Mission


def main():
    controller = MavlinkController()
    mission = Mission(controller, waypoints=WAYPOINTS)
    mission.run()


if __name__ == "__main__":
    main()
