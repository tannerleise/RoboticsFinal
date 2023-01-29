"""grocery controller."""
#pytrees is for decision trees
# Nov 2, 2022
from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard, Supervisor
import math
import numpy as np
from matplotlib import pyplot as plt
import scipy as sp
import heapq
from scipy.signal import convolve2d  # Uncomment if you want to use something else for finding the configuration space

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 2.75 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


pose_x = 0
pose_y = 0
pose_theta = 0


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

#

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

robot_parts = {}
for i, part_name in enumerate(part_names):
    robot_parts[part_name] = robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

#Getting the keyboard for Mapping
keyboard = robot.getKeyboard()
keyboard.enable(timestep)


# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None
# ------------------------------------------------------------------
# Helper Functions

#-------------------------------------------------------------------
mode = 'manual'
# mode = 'planner'
# mode = 'autonomous'

print(f"Starting robot simulation in mode: {mode}")
gripper_status="closed"

map = np.empty((1200, 1000))


if mode == 'planner':
    m = np.load("./map.npy")
    # plt.imshow(np.rot90(m))


    kernal = [[1 for _ in range(40)] for _ in range(40)]

    blurry = sp.signal.convolve2d(m, kernal, mode="same")
    c_space = []
    # Part 2.2: Compute an approximation of the “configuration space”
    for row in blurry:
        c_space.append([1 if col != 0 else 0 for col in row])

    for i in range(len(c_space)):
        for j in range(len(c_space[0])):
            if c_space[i][j] == 1:
                display.setColor(int(0xFF0000))
                display.drawPixel(i, j)

    for i in range(len(m)):
        for j in range(len(m[0])):
            if m[i][j] == 1:
                display.setColor(int(0xFFFF00))
                display.drawPixel(i, j)

    plt.imshow(c_space)
    plt.show()
    np.save("cspace.npy", np.array(c_space))


























# Main Loop
while robot.step(timestep) != -1:

    pose_y = gps.getValues()[1]
    pose_x = gps.getValues()[0]

    n = compass.getValues()
    rad = ((math.atan2(-n[0], -n[1])))  # -1.5708)
    pose_theta = rad
    # print("0/1 = ", ((math.atan2(n[0], n[1]))))
    # print("1/0 = ", ((math.atan2(n[1], n[0]))))
    # print("0/2 = ", ((math.atan2(n[0], n[2]))))
    # print("2/0 = ", ((math.atan2(n[2], n[0]))))
    # print(pose_theta)

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings) - 83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = -math.cos(alpha) * rho + 0.202
        ry = math.sin(alpha) * rho - 0.004

        # Convert detection from robot coordinates into world coordinates
        wx = math.cos(pose_theta) * rx - math.sin(pose_theta) * ry + pose_x
        wy = +(math.sin(pose_theta) * rx + math.cos(pose_theta) * ry) + pose_y

        ################ ^ [End] Do not modify ^ ##################

        # print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < LIDAR_SENSOR_MAX_RANGE:
            wx_guard = 450 + int(wx * 30)
            wy_guard = (240 - int(wy * 30))
            if (wx_guard < 900) and (wx_guard > 0) and (wy_guard > 0) and (wy_guard < 480):
                g = map[450 + int(wx * 30)][240 - int(wy * 30)]

                if g >= 1:
                    g = 1
                color = ((g * 256) ** 2) + ((g * 256) + g) * 255
                # color = (g * 256 ** 2 + g * 256 + g) * 255

                # print(color)
                if color > 0x000FFF:
                    display.setColor(int(color))
                    display.drawPixel(450 + int(wx * 30), 240 - int(wy * 30))
                if map[450 + int(wx * 30)][240 - int(wy * 30)] + 5e-3 < 1:  # guards against out of bounds
                    map[450 + int(wx * 30)][240 - int(wy * 30)] += 5e-3
    display.setColor(int(0xFF0000))

    display.drawPixel(450 + int(pose_x * 30), 240 - int(pose_y * 30))
    # print("Pixel X = ", 450 + int(wx * 30), " Pixel Y = ", 240 - int(wy * 30), " Pose X = ", int(pose_x), " Pose Y = ", int(pose_y))
    # print("Pixel X = ", 450 + int(pose_x * 12), " Pixel Y = ", 240 - int(pose_y * 15))

#Driving in manual mode
    if mode == 'manual':
        key = keyboard.getKey()
        while (keyboard.getKey() != -1): pass
        if key == keyboard.LEFT:
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            # Part 1.4: Filter map and save to filesystem
            threshold_arr = (map >= 0.5)
            threshold_arr = np.multiply(threshold_arr, 1)
            map[map < 0.5] = 0
            np.save("map.npy", threshold_arr)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else:  # slow down
            vL *= 0.75
            vR *= 0.75
    else:  # not manual mode
        # Part 3.2: Feedback controller
        # STEP 1: Calculate the error
        break
        # print("tf")

    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)



    #
    #
    # robot_parts["wheel_left_joint"].setVelocity(vL)
    # robot_parts["wheel_right_joint"].setVelocity(vR)
    #
    # if(gripper_status=="open"):
    #     # Close gripper, note that this takes multiple time steps...
    #     robot_parts["gripper_left_finger_joint"].setPosition(0)
    #     robot_parts["gripper_right_finger_joint"].setPosition(0)
    #     if right_gripper_enc.getValue()<=0.005:
    #         gripper_status="closed"
    # else:
    #     # Open gripper
    #     robot_parts["gripper_left_finger_joint"].setPosition(0.045)
    #     robot_parts["gripper_right_finger_joint"].setPosition(0.045)
    #     if left_gripper_enc.getValue()>=0.044:
    #         gripper_status="open"
