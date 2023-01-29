from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.constants import Direction
from py_trees.common import Status
import time

"""
Turn the robot's head in various directions.
"""

class TurnHeadTowardsClosestObject(Behaviour):
    def __init__(self, name=None):
        super(TurnHeadTowardsClosestObject, self).__init__(name)

        self.robot = get_robot_wrapper()
        self.start = None

    def initialise(self):
        self.robot.turn_head(self.robot.blackboard["obj_direction"])
        self.start = time.time()

    def update(self):
        # Give it a second and a half to open
        return Status.SUCCESS if time.time() - self.start > 1.5 else Status.RUNNING


class TurnHeadCenter(Behaviour):
    GRIPPER_OPEN_POS = 0.045

    def __init__(self, name=None):
        super(TurnHeadCenter, self).__init__(name)

        self.robot = get_robot_wrapper()
        self.start = None

    def initialise(self):
        self.robot.turn_head(Direction.CENTER)
        self.start = time.time()

    def update(self):
        # Give it a second and a half to open
        return Status.SUCCESS if time.time() - self.start > 1.5 else Status.RUNNING
