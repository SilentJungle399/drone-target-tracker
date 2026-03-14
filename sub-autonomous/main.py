from logger_config import setup_logging

setup_logging("sub_autonomous")

from controller import MavlinkController
from mission import Mission


def main():
    controller = MavlinkController()
    mission = Mission(controller)
    mission.run()


if __name__ == "__main__":
    main()
