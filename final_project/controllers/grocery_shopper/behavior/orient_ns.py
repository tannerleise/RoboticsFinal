from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
from py_trees.common import Status
import math

"""
Since aisles are north/south oriented, make sure to align self straight down the row before doing the grab sequence
"""
class OrientNS(Behaviour):
    def __init__(self, name=None):
        super(OrientNS, self).__init__(name)

        self.robot = get_robot_wrapper()
        self.turn_dir = None

    def initialise(self):
        # North or south based on which general direction the robot is looking
        self.turn_dir = "north" if math.pi - abs(self.robot.get_bearing_rads() - math.pi) <= (math.pi / 2) else "south"
        self.robot.parts["wheel_left_joint"].setVelocity(0.0)
        self.robot.parts["wheel_right_joint"].setVelocity(0.0)

        log(f"ORIENT: bearing: {self.robot.get_bearing_rads()} | {abs(self.robot.get_bearing_rads() - math.pi)} | {math.pi - abs(self.robot.get_bearing_rads() - math.pi)}")


    def update(self):
        log(f"Turning robot {self.turn_dir}")

        if self.turn_dir == "north":
            if math.pi - abs(self.robot.get_bearing_rads() - math.pi) > 0.01:
                if self.robot.get_bearing_rads() - math.pi >= 0:
                    self.robot.parts["wheel_left_joint"].setVelocity(0.0)

                    self.robot.parts["wheel_right_joint"].setVelocity(
                        self.robot.parts["wheel_right_joint"].getMaxVelocity() / 8)
                else:
                    self.robot.parts["wheel_right_joint"].setVelocity(0.0)

                    self.robot.parts["wheel_left_joint"].setVelocity(
                        self.robot.parts["wheel_left_joint"].getMaxVelocity() / 8)
                return Status.RUNNING
            else:
                self.robot.parts["wheel_left_joint"].setVelocity(0.0)
                self.robot.parts["wheel_right_joint"].setVelocity(0.0)
                return Status.SUCCESS
        else:
            if math.pi - (math.pi - abs(self.robot.get_bearing_rads() - math.pi)) > 0.01:
                if self.robot.get_bearing_rads() - math.pi < 0:
                    self.robot.parts["wheel_left_joint"].setVelocity(0.0)

                    self.robot.parts["wheel_right_joint"].setVelocity(
                        self.robot.parts["wheel_right_joint"].getMaxVelocity() / 8)
                else:
                    self.robot.parts["wheel_right_joint"].setVelocity(0.0)

                    self.robot.parts["wheel_left_joint"].setVelocity(
                        self.robot.parts["wheel_left_joint"].getMaxVelocity() / 8)
                return Status.RUNNING
            else:
                self.robot.parts["wheel_left_joint"].setVelocity(0.0)
                self.robot.parts["wheel_right_joint"].setVelocity(0.0)
                return Status.SUCCESS

    def terminate(self, new_status):
        log("Finished driving towards object", LogLevel.DEBUG)