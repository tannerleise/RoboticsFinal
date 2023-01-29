from core.utils import get_bearing_in_deg
from py_trees.behaviour import Behaviour
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
from py_trees.common import Status

"""
Record the starting position at the beginning of the grab sequence.
"""
class RecordStartingPos(Behaviour):
    def __init__(self, name=None):
        super(RecordStartingPos, self).__init__(name)

        self.robot = get_robot_wrapper()

    def initialise(self):
        log(f"Entered Grab Sequence.", LogLevel.INFO)
        self.robot.blackboard["grab_sequence_starting_position"] = {
            "gps": self.robot.gps.getValues(),
            "bearing_deg": get_bearing_in_deg(self.robot.compass)
        }

    def update(self):
        return Status.SUCCESS
