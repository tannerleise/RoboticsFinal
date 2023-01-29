from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
from core.constants import Direction
from py_trees.common import Status
import numpy as np
import math

"""
Turn the robot in place 90 deg towards the recorded object direction
"""

class TurnTowardsClosestObj(Behaviour):
    def __init__(self, name=None):
        super(TurnTowardsClosestObj, self).__init__(name)

        self.robot = get_robot_wrapper()

        self.start_dir = None

    def initialise(self):
        self.start_dir = np.arctan2(self.robot.compass.getValues()[1], self.robot.compass.getValues()[0])

    def update(self):
        self.robot.turn_head(Direction.CENTER)
        log(f"TURNED: {abs(np.arctan2(self.robot.compass.getValues()[1], self.robot.compass.getValues()[0]) - self.start_dir)}")

        if self.robot.blackboard["obj_direction"] == Direction.RIGHT:
            if abs(np.arctan2(self.robot.compass.getValues()[1], self.robot.compass.getValues()[0]) - self.start_dir) <= (math.pi/2) - math.pi / 9:
                self.robot.parts["wheel_right_joint"].setVelocity(0.0)

                self.robot.parts["wheel_left_joint"].setVelocity(
                    self.robot.parts["wheel_left_joint"].getMaxVelocity() / 8)
                return Status.RUNNING
            else:
                self.robot.parts["wheel_left_joint"].setVelocity(0.0)
                self.robot.parts["wheel_right_joint"].setVelocity(0.0)
                return Status.SUCCESS
        else:
            if abs(np.arctan2(self.robot.compass.getValues()[1],
                              self.robot.compass.getValues()[0]) - self.start_dir) <= (math.pi / 2) - math.pi / 9:
                self.robot.parts["wheel_left_joint"].setVelocity(0.0)

                self.robot.parts["wheel_right_joint"].setVelocity(
                    self.robot.parts["wheel_right_joint"].getMaxVelocity() / 8)
                return Status.RUNNING
            else:
                self.robot.parts["wheel_left_joint"].setVelocity(0.0)
                self.robot.parts["wheel_right_joint"].setVelocity(0.0)
                return Status.SUCCESS

    def terminate(self, new_status):
        log("Finished driving towards object", LogLevel.DEBUG)