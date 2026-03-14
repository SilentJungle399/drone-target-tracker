from logger_config import setup_logging

setup_logging("autonomous")

from controller import MavlinkController
from vision import Vision
from mission import Mission

controller = MavlinkController()
vision = Vision(dev=True, controller=controller)

mission = Mission(controller, vision)
mission.run()

vision.release()
