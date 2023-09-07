"""
    Test script for visualizing the trajectory in Gepetto Viewer.
    We will read in a set of end effector positions for the IIWA robot, and
    display the trajectory in Gepetto Viewer.
"""

import numpy as np
import pinocchio

import time

import crocoddyl
import os
print("Current Working Directory:", os.getcwd())

from robot_descriptions.loaders.pinocchio import load_robot_description

from gepetto.corbaserver import Color

robot: pinocchio.RobotWrapper = load_robot_description("iiwa_description")
robot_model = robot.model

cameraTF = [0.0, 3.0, 0.75, 0.00, 0.47, 0.50, 0.00]
display: crocoddyl.GepettoDisplay = crocoddyl.GepettoDisplay(robot, cameraTF=cameraTF, floor=False, frameNames=[])
robot.viewer.gui.refresh()


xu_traj_file_name = f"testfiles/0_0_traj.csv"
xu_traj2d = np.genfromtxt(xu_traj_file_name, delimiter=',')

eePos_traj_file_name = f"testfiles/0_0_eepos.traj"
eePos_traj2d = np.genfromtxt(eePos_traj_file_name, delimiter=',')

# create list of end effector positions
ee_pos_seq = [traj[0:3].tolist() for traj in eePos_traj2d]

# read in the trajectory files so we can get their lengths
traj_file_name_0_1 = f"testfiles/0_1_traj.csv"
traj_file_name_1_2 = f"testfiles/1_2_traj.csv"
traj_file_name_2_3 = f"testfiles/2_3_traj.csv"
traj_file_name_3_4 = f"testfiles/3_4_traj.csv"
traj_file_name_4_0 = f"testfiles/4_0_traj.csv"
traj2d_0_1 = np.genfromtxt(traj_file_name_0_1, delimiter=',')
traj2d_1_2 = np.genfromtxt(traj_file_name_1_2, delimiter=',')
traj2d_2_3 = np.genfromtxt(traj_file_name_2_3, delimiter=',')
traj2d_3_4 = np.genfromtxt(traj_file_name_3_4, delimiter=',')
traj2d_4_0 = np.genfromtxt(traj_file_name_4_0, delimiter=',')

index_0 = traj2d_0_1.shape[0]
index_1 = index_0 + traj2d_1_2.shape[0]
index_2 = index_1 + traj2d_2_3.shape[0]
index_3 = index_2 + traj2d_3_4.shape[0]
index_4 = index_3 + traj2d_4_0.shape[0]

# print out these indices
print(f"index_0: {index_0}")
print(f"index_1: {index_1}")
print(f"index_2: {index_2}")
print(f"index_3: {index_3}")
print(f"index_4: {index_4}")

# create list of these indices
target_indices = [0, index_0, index_1, index_2, index_3, index_4]

# create a list of the target positions
targets = [ee_pos_seq[i-1] for i in target_indices]

# create a sphere at each of these points
print(targets)
counter = 0
for t in targets:
    robot.viewer.gui.addSphere(
    "world/point" + str(counter), 0.05, [1.0, 0.0, 0.0, 1.0]
    )  # radius = .1, RGBA=1001
    robot.viewer.gui.applyConfiguration(
        "world/point"+ str(counter), t + [0.0, 0.0, 0.0, 1.0]
    )  # xyz+quaternion
    counter += 1

# update the display to put the robot at the starting position
q_vals = xu_traj2d[0][0:robot.nq]
display.robot.display(q_vals)

# sleep for a bit to let the display update
time.sleep(1)
key = 'iiwa_joint_7'
# color = np.hstack([np.random.choice(range(256), size=3) / 256.0, 1.0]).tolist()
color = [0.015625, 0.1484375, 0.33984375, 1.0]
print(f"using color {color}")
frameName = display.frameTrajGroup + "/" + key
display.robot.viewer.gui.addCurve(
    frameName,
    ee_pos_seq,
    color,
)
display.robot.viewer.gui.setCurveLineWidth(frameName, display.frameTrajLineWidth)
if display.fullVisibility:
    display.robot.viewer.gui.setVisibility(frameName, "ALWAYS_ON_TOP")

robot.viewer.gui.refresh()
time.sleep(1)

robot.viewer.gui.createGroup("frame")
robot.viewer.gui.addToGroup("frame", "crocoddyl")
robot.viewer.gui.addLandmark("frame", 0.01)
robot.viewer.gui.applyConfiguration("frame", [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0])
robot.viewer.gui.refresh()

# iterate over the x values
for i in range(0, len(xu_traj2d)):
    # pull out just the position values
    q_vals = xu_traj2d[i][0:robot.nq]
    # update the visualization to the new position
    display.robot.display(q_vals)
    # wait for a bit
    robot.viewer.gui.refresh()
    time.sleep(0.01)

display._addFrameCurves()
