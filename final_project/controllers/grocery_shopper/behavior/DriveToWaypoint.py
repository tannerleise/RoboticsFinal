from core.constants import MAX_SPEED, AXLE_LENGTH, WHEEL_RADIUS_M
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from core.robot import get_robot_wrapper
from core.logger import log, LogLevel
import numpy as np
import math

"""
DriveToWaypoint drives to a single waypoint each time that it is run. It utilizes a feedback controller to drive
to a coordinate. It also will interrupt itself if it see's a yellow cube within range, meaning that it will then enter
the grab sequence.
"""

class DriveToWaypoint(Behaviour):
    def __init__(self, name=None):
        super(DriveToWaypoint, self).__init__(name)

        log(f"Entered state DriveToWaypoint.", LogLevel.INFO)

        self.robot = get_robot_wrapper()
        self.first_step = True

    def initialise(self):
        self.control_gains = [.5, .8, 0.05]

    def update(self):
        log(f"Robot in state: {self.robot.current_state}")

        # Blinders means that he will drive to waypoint without stopping
        if not self.robot.blackboard["blinders"]:
            # Check if a yellow cube is close enough

            target_obj, _ = self.robot.get_target_object()
            if target_obj is not None:
                distance_to_target = np.linalg.norm([target_obj.getPosition()[0], target_obj.getPosition()[1]])
                log(f"Target object found while driving to waypoint at distance: {distance_to_target}",LogLevel.DEBUG)
                if distance_to_target < 3:
                    # Return failure to break out of the waypoint driving loop
                    return Status.FAILURE

        """
        Feedback controller
        """
        distance_error, bearing_error, heading_error = self._get_errors()
        log(f"Distance {distance_error} | Bearing {bearing_error} | Heading {heading_error}")

        instantaneous_x, instantaneous_theta = (self.control_gains[0] * distance_error,
                                                (self.control_gains[1] * bearing_error) + (
                                                            self.control_gains[2] * heading_error))

        phi_l, phi_r = ((instantaneous_x - (instantaneous_theta * AXLE_LENGTH) / 2) / WHEEL_RADIUS_M,
                        (instantaneous_x + (instantaneous_theta * AXLE_LENGTH) / 2) / WHEEL_RADIUS_M)

        log(f"Calculated phi_l/r: {phi_l}, {phi_r}")

        # Set wheel vel with clamp
        self.robot.set_wheel_vel(phi_l, phi_r, clamp=MAX_SPEED / 2)

        # Adjust gains
        if abs(bearing_error) > math.pi / 8 or math.pi * 2 - abs(bearing_error) < math.pi / 8:
            self.control_gains[0] = 0.01
            self.control_gains[1] = 1
        elif distance_error < 0.20:
            self.control_gains[0] = .9
            self.control_gains[1] = .7
        else:
            self.control_gains[0] = 0.35
            self.control_gains[1] = 0.75

        log(f"Control gains: {self.control_gains}")

        if (distance_error < 0.08 or (
                distance_error < 0.15 and math.pi - abs(bearing_error) < 0.01)):
            log(f"Waypoint reached.", LogLevel.INFO)
            self.robot.waypoint_reached()

            # Once it reaches a waypoint, if blinders is on, it will turn it off and start looking for cubes again
            # next time that it is called.
            self.robot.blackboard['blinders'] = False
            return Status.SUCCESS

        else:
            return Status.RUNNING

    def _get_errors(self):
        x_r, y_r, = self.robot.get_pos()
        x_g, y_g = self.robot.get_next_waypoint()

        distance_error = math.sqrt(math.pow((x_r - x_g), 2) + math.pow((y_r - y_g), 2))
        bearing_error = self.robot.get_bearing_error_to(x_g, y_g)
        heading_error = 0 - self.robot.get_bearing_rads()

        return distance_error, bearing_error, heading_error