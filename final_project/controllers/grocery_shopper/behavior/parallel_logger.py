from core.logger import log, LogLevel, set_log_level
from core.robot import get_robot_wrapper
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from core import constants
from enum import Enum

"""
Allow the dynamic changing of the log level while running with the following keys:
1: None
2: Error
3: Info
4: Debug

Additionally, since we were running into some issues sometimes with the robot detecting the object on the wrong side
of the aisle, we add an override with the keys R and L
"""

class Keys(Enum):
    ONE = 49
    TWO = 50
    THREE = 51
    FOUR = 52
    R = 82
    L = 76


# Starting position depends on the shelf level
class ParallelLogger(Behaviour):
    def __init__(self, name=None):
        super(ParallelLogger, self).__init__(name)

        self.robot = get_robot_wrapper()

    def initialise(self):
        pass

    def update(self):
        k = self.robot.keyboard.getKey()
        if k == -1:
            return Status.RUNNING

        if k == Keys.ONE.value:
            print("Changing the log level to NONE")
            set_log_level(LogLevel.NONE)

        elif k == Keys.TWO.value:
            print("Changing the log level to ERROR")
            set_log_level(LogLevel.ERROR)

        elif k == Keys.THREE.value:
            print("Changing the log level to INFO")
            set_log_level(LogLevel.INFO)

        elif k == Keys.FOUR.value:
            print("Changing the log level to DEBUG")
            set_log_level(LogLevel.DEBUG)

        elif k == Keys.R.value:
            log("Turning Head to the Right")
            self.robot.turn_head(constants.Direction.RIGHT)
            self.robot.blackboard['obj_direction'] = constants.Direction.RIGHT

        elif k == Keys.L.value:
            log("Turning head to the Left")
            self.robot.turn_head(constants.Direction.LEFT)
            self.robot.blackboard['obj_direction'] = constants.Direction.LEFT

        return Status.RUNNING
