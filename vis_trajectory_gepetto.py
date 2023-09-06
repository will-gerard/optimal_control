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

robot: pinocchio.RobotWrapper = load_robot_description("iiwa_description")
robot_model = robot.model

cameraTF = [2.0, 2.68, 0.54, 0.2, 0.62, 0.6, 0.22]
display: crocoddyl.GepettoDisplay = crocoddyl.GepettoDisplay(robot, cameraTF=cameraTF, floor=False, frameNames=[])
robot.viewer.gui.refresh()

xu_traj_file_name = f"testfiles/0_0_traj.csv"
xu_traj2d = np.genfromtxt(xu_traj_file_name, delimiter=',')

eePos_traj_file_name = f"testfiles/0_0_eepos.traj"
eePos_traj2d = np.genfromtxt(eePos_traj_file_name, delimiter=',')

# create list of end effector positions
ee_pos_seq = [traj[0:3].tolist() for traj in eePos_traj2d]

# update the display to put the robot at the starting position
q_vals = xu_traj2d[0][0:robot.nq]
display.robot.display(q_vals)

# sleep for a bit to let the display update
time.sleep(1)
key = 'iiwa_joint_7'
color = np.hstack([np.random.choice(range(256), size=3) / 256.0, 1.0]).tolist()
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
