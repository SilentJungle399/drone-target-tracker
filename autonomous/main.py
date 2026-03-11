from controller import MavlinkController
from vision import Vision
from mission import Mission

print("Imported modules")

controller = MavlinkController()
vision = Vision(dev=True, controller=controller)

mission = Mission(controller, vision)
mission.run()

vision.release()
