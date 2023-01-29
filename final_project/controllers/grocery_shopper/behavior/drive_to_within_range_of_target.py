from core.robot import get_robot_wrapper, Shelf
from py_trees.behaviour import Behaviour
from core.logger import log, LogLevel
from py_trees.common import Status
import numpy as np

"""
Drive forward until the object is within grabbing range.
"""


class DriveToWithinRangeOfTarget(Behaviour):
    def __init__(self, name=None):
        super(DriveToWithinRangeOfTarget, self).__init__(name)

        self.robot = get_robot_wrapper()

    def update(self):
        target_obj, shelf = self.robot.get_target_object()
        if target_obj is None:
            log("No target object found to drive to.")
            return Status.FAILURE

        distance_to_target = np.linalg.norm([target_obj.getPosition()[0], target_obj.getPosition()[1]])
        drive_target_distance = 0.65 if shelf == Shelf.TOP else 0.65

        # If the object is too far, drive forward slowly
        if distance_to_target > drive_target_distance:
            self.robot.parts["wheel_left_joint"].setVelocity(
                self.robot.parts["wheel_left_joint"].getMaxVelocity() / 10.0)
            self.robot.parts["wheel_right_joint"].setVelocity(
                self.robot.parts["wheel_right_joint"].getMaxVelocity() / 10.0)
            return Status.RUNNING

        # Object within range.
        else:
            self.robot.parts["wheel_left_joint"].setVelocity(0.0)
            self.robot.parts["wheel_right_joint"].setVelocity(0.0)
            return Status.SUCCESS

    def terminate(self, new_status):
        log("Finished driving towards object", LogLevel.DEBUG)

