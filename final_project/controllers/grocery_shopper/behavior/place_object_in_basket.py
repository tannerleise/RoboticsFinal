from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
from py_trees.common import Status

"""
Move the arm to above the basket, in preparation for dropping it.
"""

class PlaceObjectInBasket(Behaviour):
    BASKET_POSITION = [0.1, 0, -0.4]

    def __init__(self, name=None):
        super(PlaceObjectInBasket, self).__init__(name)

        self.robot = get_robot_wrapper()

    def initialise(self):
        log(f"Moving arm to basket.", LogLevel.INFO)

        self.robot.ik_handler.reach_arm_to_target_camera_coords(PlaceObjectInBasket.BASKET_POSITION)

    def update(self):
        if self.robot.ik_handler.arm_at_target(PlaceObjectInBasket.BASKET_POSITION):
            log(f"Arm is at basket", LogLevel.INFO)
            return Status.SUCCESS
        else:
            log("Arm is still moving to basket", LogLevel.DEBUG)
            return Status.RUNNING

    def terminate(self, new_status):
        log("Finished moving robot arm to starting position", LogLevel.DEBUG)
