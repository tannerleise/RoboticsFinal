from enum import Enum

UPPER_STARTING_POSITION = [0.5, 0, 0.1]
LOWER_STARTING_POSITION = [0.25, 0, -0.5]
TELE_ADJUSTMENT = 0.01
GRIPPER_OPEN_POS = 0.045

MAX_SPEED = 7.0
MAX_SPEED_MS = 0.633  # [m/s]
AXLE_LENGTH = 0.4044  # m
WHEEL_RADIUS_M = MAX_SPEED_MS / MAX_SPEED

MOTOR_LEFT = "wheel_left_joint"
MOTOR_RIGHT = "wheel_right_joint"

PART_NAMES = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
              "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint")

SHELF_HEIGHT_PADDING = 0.2


class Direction(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    CENTER = "CENTER"
