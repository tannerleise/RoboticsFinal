from core.constants import UPPER_STARTING_POSITION, LOWER_STARTING_POSITION
from core.robot import get_robot_wrapper, Shelf
from py_trees.behaviour import Behaviour
from core.logger import log, LogLevel
from py_trees.common import Status

"""
The starting position depends on the level of the detected object (if one is detected).

A higher starting position will be used for objects on the top shelf, a lower starting position for objects on the
middle shelf
"""

class MoveArmToStartingPosition(Behaviour):
    def __init__(self, name=""):
        super(MoveArmToStartingPosition, self).__init__(name)

        self.robot = get_robot_wrapper()

        # Hardcoded top or middle starting position.
        if name.lower() == "top":
            self.starting_pos = UPPER_STARTING_POSITION
        elif name.lower() == "middle":
            self.starting_pos = LOWER_STARTING_POSITION
        else:
            self.starting_pos = None


    def initialise(self):
        log("Moving arm to starting position")

        # Nothing was hardcoded
        if self.starting_pos is None:
            _, shelf = self.robot.get_target_object()
            self.starting_pos = UPPER_STARTING_POSITION if shelf == Shelf.TOP else LOWER_STARTING_POSITION

        self.robot.ik_handler.reach_arm_to_target_camera_coords(self.starting_pos)

    def update(self):
        log(self.starting_pos)

        if self.robot.ik_handler.arm_at_target(self.starting_pos):
            return Status.SUCCESS
        else:
            log("Arm is still moving to starting position", LogLevel.DEBUG)
            return Status.RUNNING

    def terminate(self, new_status):
        log("Finished moving robot arm to starting position", LogLevel.DEBUG)
