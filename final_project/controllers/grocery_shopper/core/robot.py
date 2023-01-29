from core.constants import Direction, MOTOR_LEFT, MOTOR_RIGHT
from core.logger import log, LogLevel
from core.ik import IKHandler
from controller import Robot
from enum import Enum
import numpy as np
import math
from core.object_recognition_ML import predict

class RobotConstants(Enum):
    MAX_SPEED = 7.0
    MAX_SPEED_MS = 0.633  # [m/s]
    AXLE_LENGTH = 0.4044  # m
    MOTOR_LEFT = 10
    MOTOR_RIGHT = 11
    N_PARTS = 12
    LIDAR_ANGLE_BINS = 667
    LIDAR_SENSOR_MAX_RANGE = 5.5  # Meters
    LIDAR_ANGLE_RANGE = math.radians(240)


class Shelf(Enum):
    TOP = "TOP"
    MIDDLE = "MIDDLE"


"""
Class instatiated with the "get_robot_wrapper" function below.
"""
class _Robot:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_state = 1

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
                           "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
                           "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint",
                           "gripper_left_finger_joint", "gripper_right_finger_joint")

        self.camera = self._init_camera()
        self.gps, self.compass = self._init_navigation()
        self.lidar = self._init_lidar()
        self.keyboard = self._init_keyboard()
        self.display = self.robot.getDevice("display")
        self.parts = self._init_parts()
        self.ik_handler = IKHandler("robot_urdf.urdf", self.robot, self.timestep)

        self.blackboard = {'blinders': False}

    def get_target_object(self, use_ml=False):
        """
        use_ml is turned to False for debugging since it's much faster, but the ML is fully functional.

        For the debug version, it just finds all the objects, uses the getcolors to see if it's yellow, and then returns
        the closest object as well as what shelf it's on (since that effects the adjustments for moving the arm)
        """
        if not use_ml:
            objects = self.camera.getRecognitionObjects()

            closest_yellow = None
            for i, o in enumerate(objects):
                if o.getColors()[0] == 1 and o.getColors()[1] == 1:
                    log(f"{o.getId()} | {o.getPosition()}")

                    if closest_yellow is None:
                        closest_yellow = o
                    else:
                        curr_closest_pos = np.array(closest_yellow.getPosition())
                        err_1 = np.linalg.norm(curr_closest_pos)

                        challenger_pos = np.array(o.getPosition())
                        err_2 = np.linalg.norm(challenger_pos)

                        closest_yellow = o if err_2 < err_1 else closest_yellow

            if closest_yellow is None:
                return None, None

            # classification = predict(closest_yellow)
            return closest_yellow, Shelf.TOP if closest_yellow.getPosition()[2] > -0.1 else Shelf.MIDDLE
        else:
            # TODO
            return None, None

    def turn_head(self, direction: Direction):
        head_joint = self.robot.getDevice("head_1_joint")
        head_joint.setVelocity(head_joint.getMaxVelocity() / 2)

        if direction == Direction.CENTER:
            head_joint.setPosition(0)
        else:
            d = 1 if direction == Direction.LEFT else -1
            position = d * head_joint.getMaxPosition()
            head_joint.setPosition(position)

    def get_bearing_rads(self):
        n = self.compass.getValues()
        rad = ((math.atan2(n[0], n[1])))
        return rad % (2 * math.pi)

    def set_wheel_vel(self, phi_l, phi_r, clamp=None):
        if clamp is not None:
            if phi_l < -clamp:
                phi_l = -clamp
            elif phi_l > clamp:
                phi_l = clamp

            if phi_r < -clamp:
                phi_r = -clamp
            elif phi_r > clamp:
                phi_r = clamp

        self.parts[MOTOR_RIGHT].setVelocity(phi_r)
        self.parts[MOTOR_LEFT].setVelocity(phi_l)

    def get_pos(self):
        return (self.gps.getValues()[0], self.gps.getValues()[1])

    def get_bearing_error_to(self, waypoint_x, waypoint_y):
        """
        Get the bearing error to a world coordinate and account for jumps such as 0 -> 2pi
        """
        x_r, y_r = self.get_pos()
        bear = math.atan2(waypoint_y - y_r, waypoint_x - x_r)
        bearing_error = (bear - self.get_bearing_rads())

        if bearing_error > math.pi:
            bearing_error -= math.pi * 2

        if bearing_error < -math.pi:
            bearing_error += math.pi * 2

        return bearing_error

    def get_next_waypoint(self):
        return self.waypoints[self.current_state + 1]

    def waypoint_reached(self):
        self.current_state += 1

    def object_at_horiz_center(self, object):
        """
        Is the object at the center of the camera?
        """
        half = self.camera.getWidth() // 2
        obj_x, _ = object.getPositionOnImage()
        obj_w, _ = object.getSizeOnImage()
        return half == obj_x + obj_w


    def _init_camera(self):
        camera = self.robot.getDevice('camera')
        camera.enable(self.timestep)
        camera.recognitionEnable(self.timestep)
        return camera

    def _init_navigation(self):
        gps = self.robot.getDevice("gps")
        gps.enable(self.timestep)
        compass = self.robot.getDevice("compass")
        compass.enable(self.timestep)
        return gps, compass

    def _init_lidar(self):
        lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        lidar.enable(self.timestep)
        lidar.enablePointCloud()
        return lidar

    def _init_keyboard(self):
        keyboard = self.robot.getKeyboard()
        keyboard.enable(self.timestep)
        return keyboard

    def _init_parts(self):
        # All motors except the wheels are controlled by position control. The wheels
        # are controlled by a velocity controller. We therefore set their position to infinite.
        target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf', 0.045, 0.045)

        robot_parts = {}
        for i, part_name in enumerate(self.part_names):
            log(f"Iniitializing robot part: {part_name}", LogLevel.DEBUG)
            robot_parts[part_name] = self.robot.getDevice(part_name)
            robot_parts[part_name].setPosition(float(target_pos[i]))
            robot_parts[part_name].setVelocity(0.0)

        return robot_parts


"""
SINGLETON ROBOT TO SHARE
"""

_robot_instance = None


def get_robot_wrapper():
    global _robot_instance
    if _robot_instance is None:
        _robot_instance = _Robot([(-3, 3), (5.3, 5.8), (8.99, 5.8), (5.0, 5.8), (-1, 2), (13, 2), (12.5, -2), (4.6, -2), (-5.2, -2.14)])
    return _robot_instance
