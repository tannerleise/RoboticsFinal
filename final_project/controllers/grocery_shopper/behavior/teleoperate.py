from core.constants import LOWER_STARTING_POSITION
from core.robot import get_robot_wrapper
from py_trees.behaviour import Behaviour
from core.logger import log, LogLevel
from py_trees.common import Status
from core import constants
from enum import Enum

"""
Teleoperate the arm.

Operations:

X-PLANE
- up    -> forward
- down  -> back
- left  -> left
- right -> right

Z-PLANE
- w -> up
- s -> down

GRIPPER
- o -> open
- c -> close

Press ENTER to finish and move to the next state.

"""

class Keys(Enum):
    NO_KEY = -1
    UP = 315
    LEFT = 314
    RIGHT = 316
    DOWN = 317
    SPACE = 32
    W = 87
    A = 65
    S = 83
    D = 68
    O = 79
    C = 67
    ENTER = 4
    FIVE = 53
    SIX = 54
    SEVEN = 55
    EIGHT = 56
    NINE = 57
    ZERO = 48

class Teleoperate(Behaviour):
    def __init__(self, name=None):
        self.hardcoded_coords = name
        super(Teleoperate, self).__init__(name)

        self.robot = get_robot_wrapper()
        self.coords = constants.UPPER_STARTING_POSITION

    def initialise(self):
        log("Teleoperating Arm", LogLevel.INFO)

        # Can put coords into the name field
        if self.hardcoded_coords:
            self.coords = list(map(lambda x: float(x), self.hardcoded_coords.split(",")))

        # Otherwise base it on the target object
        else:
            obj, shelf = self.robot.get_target_object()
            self.coords = LOWER_STARTING_POSITION if not obj else obj.getPosition()

        self.starting = [self.coords[0], self.coords[1], self.coords[2]]
        self.orientation = [0, 0, 1]

    def update(self):
        key = self.robot.keyboard.getKey()
        log(key)
        if key == Keys.NO_KEY.value:
            return Status.RUNNING

        # X PLANE
        if key == Keys.RIGHT.value:
            self.coords[1] -= constants.TELE_ADJUSTMENT
        elif key == Keys.LEFT.value:
            self.coords[1] += constants.TELE_ADJUSTMENT
        elif key == Keys.UP.value:
            self.coords[0] += constants.TELE_ADJUSTMENT
        elif key == Keys.DOWN.value:
            self.coords[0] -= constants.TELE_ADJUSTMENT

        # Z PLANE
        elif key == Keys.W.value:
            self.coords[2] += constants.TELE_ADJUSTMENT
        elif key == Keys.S.value:
            self.coords[2] -= constants.TELE_ADJUSTMENT

        elif key == Keys.ZERO.value:
            log(f"Resetting teleoperation from {self.coords} to {self.starting}.")
            self.coords = self.starting

        # GRIPPER
        elif key == Keys.O.value:
            self.robot.parts["gripper_left_finger_joint"].setVelocity(
                self.robot.parts["gripper_left_finger_joint"].getMaxVelocity())
            self.robot.parts["gripper_right_finger_joint"].setVelocity(
                self.robot.parts["gripper_right_finger_joint"].getMaxVelocity())
            self.robot.parts["gripper_left_finger_joint"].setPosition(constants.GRIPPER_OPEN_POS)
            self.robot.parts["gripper_right_finger_joint"].setPosition(constants.GRIPPER_OPEN_POS)
        elif key == Keys.C.value:
            self.robot.parts["gripper_left_finger_joint"].setVelocity(
                self.robot.parts["gripper_left_finger_joint"].getMaxVelocity() / 2.0)
            self.robot.parts["gripper_right_finger_joint"].setVelocity(
                self.robot.parts["gripper_right_finger_joint"].getMaxVelocity() / 2.0)
            self.robot.parts["gripper_left_finger_joint"].setPosition(0)
            self.robot.parts["gripper_right_finger_joint"].setPosition(0)
        elif key == Keys.ENTER.value:
            return Status.SUCCESS

        try:
            self.robot.ik_handler.reach_arm_to_target_camera_coords(self.coords, target_orientation=self.orientation)
        except Exception as e:
            log(f"error {e}")

        return Status.RUNNING

    def terminate(self, new_status):
        log("Finished moving robot arm to starting position", LogLevel.DEBUG)
