from core.robot import get_robot_wrapper
from py_trees.behaviour import Behaviour
from core.logger import log, LogLevel
from py_trees.common import Status

"""
Move the grabber to above the detected yellow cube.
"""


class MoveArmToAboveTarget(Behaviour):
    def __init__(self, name=None):
        super(MoveArmToAboveTarget, self).__init__(name)

        self.robot = get_robot_wrapper()

    def update(self):
        log("Moving arm to above the target")

        target_obj, shelf = self.robot.get_target_object()
        if target_obj is None:
            log("No target object found to reach to.")
            return Status.FAILURE

        loc = target_obj.getPosition()

        # Slight adjustments to where we want to put the grabber in relation to the target
        loc[2] -= 0.01
        loc[0] -= 0.1

        if not self.robot.ik_handler.arm_at_target(loc):
            self.robot.ik_handler.reach_arm_to_target_camera_coords(loc)
            return Status.RUNNING
        else:
            return Status.SUCCESS

    def terminate(self, new_status):
        log("Finished moving robot arm to above the object", LogLevel.DEBUG)
