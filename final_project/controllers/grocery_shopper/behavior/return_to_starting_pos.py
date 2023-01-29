from core.robot import get_robot_wrapper
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from core.logger import log

"""
Return to the last waypoint (so we don't miss any cubes during our grab sequence), and make sure not to have
that return to the waypoint interrupted on the way back by enabling the blinders.
"""

class ReturnToStartingPos(Behaviour):
    def __init__(self, name=None):
        super(ReturnToStartingPos, self).__init__(name)

        self.robot = get_robot_wrapper()
        self.start = None

    def initialise(self):
        log("Returning to starting position")
        self.robot.current_state -= 1
        self.robot.blackboard['blinders'] = True

    def update(self):
        return Status.SUCCESS