from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
from core.constants import Direction
from py_trees.common import Status
import numpy as np
import math

"""
Use the bearing of the object relative to the robot to determine if the object it is looking at is to the left
of to the right.
"""

class DetermineObjectDirection(Behaviour):
    def __init__(self, name=None):
        super(DetermineObjectDirection, self).__init__(name)

        self.robot = get_robot_wrapper()

    def update(self):
        obj, shelf = self.robot.get_target_object()
        rad = np.arctan2(obj.getPosition()[0], obj.getPosition()[1]) - math.pi / 2
        self.robot.blackboard["obj_direction"] = Direction.LEFT if rad < 0 else Direction.RIGHT

        log(f"Object detected at rad {rad}")
        log(f"Determined that the closest object is: {self.robot.blackboard['obj_direction']}", LogLevel.DEBUG)
        return Status.SUCCESS