from core.robot import get_robot_wrapper
from py_trees.behaviour import Behaviour
from core.logger import log, LogLevel
from py_trees.common import Status

"""
Drive forward until the object is centered on the robots camera.
"""


class DrivingForwardUntilObjCentered(Behaviour):
    def __init__(self, name=None):
        super(DrivingForwardUntilObjCentered, self).__init__(name)

        self.robot = get_robot_wrapper()

    def update(self):
        target_obj, shelf = self.robot.get_target_object()
        if target_obj is None:
            log("No target object found to drive to.")
            return Status.FAILURE

        # If object not centered, drive forward.
        if not self.robot.object_at_horiz_center(self.robot.get_target_object()[0]):
            self.robot.parts["wheel_left_joint"].setVelocity(
                self.robot.parts["wheel_left_joint"].getMaxVelocity() / 4)
            self.robot.parts["wheel_right_joint"].setVelocity(
                self.robot.parts["wheel_right_joint"].getMaxVelocity() / 4)
            return Status.RUNNING

        # Stop if centered
        else:
            self.robot.parts["wheel_left_joint"].setVelocity(0.0)
            self.robot.parts["wheel_right_joint"].setVelocity(0.0)
            return Status.SUCCESS

    def terminate(self, new_status):
        log("Finished driving towards object", LogLevel.DEBUG)