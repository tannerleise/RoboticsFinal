from core.constants import PART_NAMES, SHELF_HEIGHT_PADDING
from core.logger import log, LogLevel
from ikpy.chain import Chain
import numpy as np

class IKHandler:
    def __init__(self, urdf_path: str, robot, timestep):

        self.chain = Chain.from_urdf_file(urdf_path, last_link_vector=[0.004, 0, -0.1741],
                                          base_elements=["base_link", "base_link_Torso_joint", "Torso",
                                                         "torso_lift_joint", "torso_lift_link",
                                                         "torso_lift_link_TIAGo front arm_11367_joint",
                                                         "TIAGo front arm_11367"])

        self.robot = robot
        self.timestep = timestep
        self._deactivate_uncontrollable_links()
        self.motors = self._initialize_motors()

        self.current_target = [0 for i in range(12)]

        self.cache = {}

    def move_arm(self, ik_results):
        for i in range(len(ik_results)):
            if self.chain.links[i].name in PART_NAMES:
                self.robot.getDevice(self.chain.links[i].name).setPosition(ik_results[i])

    def arm_at_target(self, camera_coords, shelf_adjusted=True, err=0.0005, orientation="Y"):
        """
        Check to see if the arm has reached the targeted camera coordinates. If err within margin, say it's ok.
        """
        robot_coords = self._camera_to_shelf_adjusted_robot_coords(camera_coords) \
            if shelf_adjusted else self._camera_to_robot_coords(camera_coords)

        pos, ik = self.get_position(), self.get_ik(robot_coords, orientation)

        log(f"\nPOS: {pos}\nIK:  {ik}\n")
        log(f"ERR: {self._calculate_error(pos, ik)}")

        if self._calculate_error(pos, ik) <= err:
            log(f"Arm is at target", LogLevel.INFO)
            return True
        else:
            return False

    def get_position(self):
        """
        Get the position of all of the joints that we are trying to control.
        """
        pos = [0, 0, 0, 0.0] + [m.getPositionSensor().getValue() for m in self.motors] + [0, 0, 0, 0]
        log(f"Found current pos as {pos} | len {len(pos)}")
        return pos

    def get_ik(self, target, orientation="Y", target_orientation=[0, 0, 1]):
        """
        Get the IK for the robot coordinates. Use a cached value if available.
        """
        # in robot coords
        # Only support caching for default orientation
        cached_result = None if not np.array_equal(np.array(target_orientation),
                                                   np.array([0, 0, 1])) else self._get_cached_ik_result(target)

        if cached_result is not None:
            return cached_result

        try:
            # No cache found, calculate IK and cache the new value
            result = self.chain.inverse_kinematics(target, initial_position=self.get_position(),
                                                   target_orientation=target_orientation, orientation_mode=orientation)
            self.cache['ik_result'] = (target, result)

            return result
        except ValueError as e:
            log(f"Robot arm is attempting to move to infeasible location {target}: {e}", LogLevel.ERROR)

            # If in infeasible location, just return an old result I guess
            if self.cache.get('ik_result') is not None:
                log("USING STALE IK RESULT")
                return self.cache['ik_result'][1]

            # Otherwise there's nothing we can do
            else:
                raise e

    def reach_arm_to_target_camera_coords(self, camera_coords, shelf_adjusted=True, orientation="Y",
                                          target_orientation=[0, 0, 1]):
        log(f"Moving robot arm to camera coordinates {camera_coords}", LogLevel.DEBUG)

        robot_coords = self._camera_to_shelf_adjusted_robot_coords(
            camera_coords) if shelf_adjusted else self._camera_to_robot_coords(camera_coords)

        self.move_arm(self.get_ik(robot_coords, orientation, target_orientation))

    def _deactivate_uncontrollable_links(self):
        # Workaround because for some reason the torso sinks into the ground if you don't move it
        self.robot.getDevice('torso_lift_joint').setVelocity(0.1)
        self.robot.getDevice('torso_lift_joint').setPosition(0.001)

        for link_id in range(len(self.chain.links)):
            link = self.chain.links[link_id]
            if link.name not in PART_NAMES or link.name == "torso_lift_joint":
                log("Disabling {}".format(link.name), LogLevel.DEBUG)
                self.chain.active_links_mask[link_id] = False

    def _initialize_motors(self):
        motors = []
        for link in self.chain.links:
            if link.name in PART_NAMES and link.name != "torso_lift_joint":
                motor = self.robot.getDevice(link.name)

                motor.setVelocity(self.robot.getDevice(link.name).getMaxVelocity() / 4.0)

                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timestep)

                motors.append(motor)
        return motors

    def _camera_to_shelf_adjusted_robot_coords(self, camera_coords):
        """
        The camera frame of reference is slightly above the robot frame of reference
        """
        coords = self._camera_to_robot_coords(camera_coords)
        coords[2] += SHELF_HEIGHT_PADDING
        return coords

    def _camera_to_robot_coords(self, camera_coords):
        """
        Values found via trial and error
        """
        return [(camera_coords[0]) + 0.22, camera_coords[1] + 0.08, (camera_coords[2]) + 0.97]

    def _calculate_error(self, target, position):
        a = np.array(target)
        b = np.array(position)
        return np.linalg.norm(b - a)

    def _get_cached_ik_result(self, target):
        """
        Return the cached result if it's computed for a position that's very close to the new position that we are
        trying to compute. Otherwise, say there's nothing cached.
        """
        if self.cache.get('ik_result') is None:
            return None
        ik_computed, ik_result = self.cache['ik_result']
        err = self._calculate_error(ik_computed, np.array(target))
        if np.array_equal(np.array(target), np.array(ik_computed)) or err < 0.001:
            return ik_result
        else:
            log(f"No cache found for target: target {target} | cached {ik_computed} | err {err}", LogLevel.DEBUG)
            return None
