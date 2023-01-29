from behavior.driving_forward_until_obj_centered import DrivingForwardUntilObjCentered
from behavior.drive_to_within_range_of_target import DriveToWithinRangeOfTarget
from behavior.move_arm_to_starting_position import MoveArmToStartingPosition
from behavior.turn_head import TurnHeadTowardsClosestObject, TurnHeadCenter
from behavior.determine_object_direction import DetermineObjectDirection
from py_trees.decorators import FailureIsSuccess, SuccessIsRunning
from behavior.move_arm_to_above_target import MoveArmToAboveTarget
from behavior.place_object_in_basket import PlaceObjectInBasket
from behavior.return_to_starting_pos import ReturnToStartingPos
from behavior.record_starting_position import RecordStartingPos
from behavior.turn_in_place import TurnTowardsClosestObj
from behavior.back_up_to_starting_dist import BackUp
from behavior.DriveToWaypoint import DriveToWaypoint
from behavior.parallel_logger import ParallelLogger
from py_trees.composites import Sequence, Parallel
from behavior.teleoperate import Teleoperate
from py_trees.common import ParallelPolicy
from core.robot import get_robot_wrapper
from behavior.orient_ns import OrientNS
from core.logger import log, LogLevel
from core.rrt import world_setup
from numpy import load
"""
So generally speaking, our path forward should be as follows:
- Create a root sequence for the robot which involves going through the isles
- When the robot sees a yellow object (see core/robot.py - get_target_object(), which needs to be implemented correctly)
  it should line itself up perpendicularly with it, so that the robot is directly facing it. The manipulation will not
  work otherwise!
- The root sequence should branch to grab_sequence
- When grab_sequence is completed, it should traverse back to the next waypoint and continue going through the isles.
"""

log("Initializing the controller.", LogLevel.INFO)
log("Initializing the behavior tree.", LogLevel.INFO)

robot = get_robot_wrapper()

teleoperation = Sequence("Teleoperation", children=[
    MoveArmToStartingPosition(),
    Teleoperate()
])
teleoperation.setup_with_descendants()

driving_sequence = Sequence("DrivingForward", children=[DriveToWaypoint()])

driving_sequence.setup_with_descendants()

main_sequence = Sequence("DrivingInterrupter")
main_sequence.add_children([

    # Start with the arm at the top position
    MoveArmToStartingPosition("top"),

    # Drive between waypoints
    # Success is running:
    #   hits success every time that a waypoint is reached, but we want to keep it following waypoints until interrupted
    # Failure is success:
    #   a failure is triggered when he sees a yellow object close, so we want to go into the grab sequence
    FailureIsSuccess(SuccessIsRunning(driving_sequence)),

    # GRAB SEQUENCE: Prelude
    RecordStartingPos(),
    DetermineObjectDirection(),

    # GRAB SEQUENCE: Setup - try to standardise his pose
    MoveArmToStartingPosition(),
    OrientNS(),

    # GRAB SEQUENCE: Turn - turn towards the object
    TurnHeadTowardsClosestObject(),
    DrivingForwardUntilObjCentered(),
    TurnHeadCenter(),
    TurnTowardsClosestObj(),

    # GRAB SEQUENCE: Approach - get close to the object
    MoveArmToStartingPosition(),
    DriveToWithinRangeOfTarget(),

    # GRAB SEQUENCE: Positioning - Move arm to the object and teleoperate
    MoveArmToAboveTarget(),
    Teleoperate(),

    # GRAB SEQUENCE: Finalize - clear the shelf by backing up, put object in basket
    BackUp(),
    MoveArmToStartingPosition("Middle"),
    PlaceObjectInBasket(),
    Teleoperate("0.1,0,-0.4"), # Hardcode basket coords

    # GRAB SEQUENCE: End
    ReturnToStartingPos(),

])

# Monitor for key presses while we run the main sequence
simulation = Parallel("RobotSim", policy=ParallelPolicy.SuccessOnOne())
simulation.add_children([
    main_sequence,
    ParallelLogger(),
]
)
main_sequence.setup_with_descendants()
simulation.setup_with_descendants()

"""
RRT
"""

default_route = [((-5.9,5.8) ,(4.48, 5.7)),
   ((4.48, 5.7),(12.6, 6.06)),
 ((12.6, 6.06) ,(12.6, 2.25)),
 ((12.6, 2.25) ,(-6.23, 1.98)),
 ((-6.23, 1.98) ,(-6.18, -1.92)),
 ((-6.18, -1.92), (12.8, -1.64)),
 ((12.8, -1.64) ,(12.9, -5.02)),
 ((12.9, -5.02),(7.35263,-5.92107)),
 ((7.35263,-5.92107), (-0.174304, -5.94245)),
 ((-0.174304, -5.94245),(-4.80645,-5.90157))
                 ]

# Sets up the RRT world coords for path generation
path = []

# # RRT Generates Nodes waypoints around the store for robot to explore each aisle
# for start_pos, end_pos in default_route:
#     all_waypoints = rrt(bounds, valid, start_pos, end_pos, 7000, 50)
#     waypoints = get_path(all_waypoints, end_pos)
#     for node in waypoints:
#         path.append(node)
# arr_to_save = np.array(path)
# save('route2.npy', arr_to_save)
path = load('route2.npy')

# plt.scatter(path[:, 0], path[:, 1])
# plt.show()
# Waypoints given to robot storage

robot.waypoints = [(0, 0)] + path

# Main Logic Initiation
while robot.robot.step(robot.timestep) != -1:
    if(robot.current_state < len(path) - 1):
        # driving_sequence.tick_once()
        simulation.tick_once()
    else:
        robot.set_wheel_vel(0,0)
