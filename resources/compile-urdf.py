import math
import pybullet as p
import time
import pybullet_data
import numpy as np

# Create the bullet physics engine environment
from gym_pupper.common.urdf_helpers import URDF

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.resetDebugVisualizerCamera(
    cameraDistance=0.45, cameraYaw=135, cameraPitch=-45, cameraTargetPosition=[0, 0, 0]
)
p.setGravity(0, 0, -10)  # good enough
frequency = 240  # Hz
p.setTimeStep(1 / frequency)
p.setRealTimeSimulation(0)


# This loads the checkerboard background
p.loadURDF("plane.urdf")

# Robot model starting position
startPos = [0, 0, 0]  # xyz
startOrientation = p.getQuaternionFromEuler(
    [0, 0, 0]
)  # rotated around which axis? # np.deg2rad(90)

# Actually load the URDF file into simulation, make the base of the robot unmoving
# robot = p.loadURDF("./ergourdf.urdf.xml", startPos, startOrientation, useFixedBase=1)
urdf = URDF("rx150.xacro.xml")
# urdf = URDF("a1-barebones.urdf.xml")
robot = p.loadURDF(
    urdf.path, startPos, startOrientation, useFixedBase=1, useMaximalCoordinates=0
)
# robot = p.loadMJCF("../gym_pupper/assets/pupper_pybullet_out.xml")[1]

motors = []

# Container for debug inputs
debugParams = []

for i in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, i)
    print(joint_info)
    joint_name = joint_info[1].decode()
    # if "_joint" == joint_name[-6:]:
    if joint_name in [
        "waist",
        "shoulder",
        "elbow",
        "wrist_angle",
        "wrist_rotate",
        "left_finger",
        "right_finger",
    ]:
        j_min = joint_info[8]
        j_max = joint_info[9]
        j_center = (j_min + j_max) / 2
        motor = p.addUserDebugParameter(joint_name, j_min, j_max, j_center)
        motors.append(i)
        debugParams.append(motor)

# motors = [6, 8, 9, 11, 13, 14, 16, 18, 19, 21, 23, 24]
# motors = [0, 1, 2]


# In the user interface, create a slider for each motor to control them separately.
# for i in range(len(motors)):


start = time.time()

# Stepping frequency * 30 = run the simulation for 30 seconds
for i in range(frequency * 30):
    motorPos = []

    for i in range(len(motors)):
        # pos = (math.pi / 2) * p.readUserDebugParameter(debugParams[i])
        pos = p.readUserDebugParameter(debugParams[i])
        motorPos.append(pos)
        p.setJointMotorControl2(
            robot, motors[i], p.POSITION_CONTROL, targetPosition=pos
        )

    p.stepSimulation()

    time.sleep(1.0 / frequency)

print(time.time() - start)

p.disconnect()
