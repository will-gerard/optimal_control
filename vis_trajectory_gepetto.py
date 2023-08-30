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
display: crocoddyl.GepettoDisplay = crocoddyl.GepettoDisplay(robot, cameraTF=cameraTF, floor=False)
robot.viewer.gui.refresh()

xu_traj_file_name = f"testfiles/0_0_traj.csv"
xu_traj2d = np.genfromtxt(xu_traj_file_name, delimiter=',')

# update the display to put the robot at the starting position
q_vals = xu_traj2d[0][0:robot.nq]
display.robot.display(q_vals)

# sleep for a bit to let the display update
time.sleep(1)

# iterate over the x values
for i in range(0, len(xu_traj2d)):
    # pull out just the position values
    q_vals = xu_traj2d[i][0:robot.nq]
    # update the visualization to the new position
    display.robot.display(q_vals)
    # wait for a bit
    robot.viewer.gui.refresh()
    time.sleep(0.01)


