from core.robot import get_robot_wrapper
from py_trees.behaviour import Behaviour
from core.logger import log, LogLevel
from py_trees.common import Status
import time

"""
Back the robot up for 5 seconds, to clear the shelf.
"""
class BackUp(Behaviour):
    def __init__(self, name=None):
        super(BackUp, self).__init__(name)

        self.robot = get_robot_wrapper()

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if time.time() - self.start_time < 5:
            # Reverse
            self.robot.parts["wheel_left_joint"].setPosition(float("-inf"))
            self.robot.parts["wheel_left_joint"].setVelocity(-
                self.robot.parts["wheel_left_joint"].getMaxVelocity() / 10.0)
            self.robot.parts["wheel_right_joint"].setPosition(float("-inf"))
            self.robot.parts["wheel_right_joint"].setVelocity(-
                self.robot.parts["wheel_right_joint"].getMaxVelocity() / 10.0)

            return Status.RUNNING
        else:

            # Stop moving
            self.robot.parts["wheel_left_joint"].setVelocity(0.0)
            self.robot.parts["wheel_right_joint"].setVelocity(0.0)
            return Status.SUCCESS

    def terminate(self, new_status):
        log("Finished driving towards object", LogLevel.DEBUG)

