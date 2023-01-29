from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
from py_trees.common import Status
from time import time

"""
Close the gripper around a cube.
"""
class CloseGripper(Behaviour):

    def __init__(self, name):
        super(CloseGripper, self).__init__(name)

        self.robot = get_robot_wrapper()

    def initialise(self):
        self.robot.parts["gripper_left_finger_joint"].setVelocity(
            self.robot.parts["gripper_left_finger_joint"].getMaxVelocity() / 2.0)
        self.robot.parts["gripper_right_finger_joint"].setVelocity(
            self.robot.parts["gripper_right_finger_joint"].getMaxVelocity() / 2.0)
        self.robot.parts["gripper_left_finger_joint"].setPosition(0)
        self.robot.parts["gripper_right_finger_joint"].setPosition(0)
        self.starting_time = time()

    def update(self):
        # Give it a second and a half to open
        sec_elapsed = time() - self.starting_time
        if sec_elapsed > 2:
            return Status.SUCCESS
        else:
            return Status.RUNNING

    def terminate(self, new_status):
        log("Gripper has been closed.", LogLevel.DEBUG)
