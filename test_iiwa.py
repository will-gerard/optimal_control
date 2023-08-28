import numpy as np
import pinocchio

import example_robot_data

import crocoddyl
import os
print("Current Working Directory:", os.getcwd())

from robot_descriptions.loaders.pinocchio import load_robot_description

robot: pinocchio.RobotWrapper = load_robot_description("iiwa_description")
robot_model = robot.model

q = np.zeros(robot.nq)
pinocchio.forwardKinematics(robot.model, robot.data, q)

# joint 7 is the final joint in iiwa
# executed forward kinematics on the robot from some random starting configurations,
# validated that the position of joint 7 is the same as the position of the end effector from when 
# grid compute end effector location is run from the same starting position
joint_7_index = robot.model.getFrameId("iiwa_joint_7")

DT = 1e-3
KNOT_POINTS = 32
NUM_CONTROLS = 7

cameraTF = [2.0, 2.68, 0.54, 0.2, 0.62, 0.72, 0.22]
display = crocoddyl.GepettoDisplay(robot, cameraTF=cameraTF, floor=False)
robot.viewer.gui.refresh()

# read in test files

# Read CSV files into numpy arrays
eePos_traj_file_name = f"testfiles/0_0_eepos.traj"
xu_traj_file_name = f"testfiles/0_0_traj.csv"

eePos_traj2d = np.genfromtxt(eePos_traj_file_name, delimiter=',')
xu_traj2d = np.genfromtxt(xu_traj_file_name, delimiter=',')

# Check length < knot_points
if len(eePos_traj2d) < KNOT_POINTS:
    print("precomputed traj length < knotpoints, not implemented")
    exit(1)

# Flatten the 2D arrays to 1D
h_eePos_traj = eePos_traj2d.flatten()
h_xu_traj = xu_traj2d.flatten()

print(h_eePos_traj)
print(h_xu_traj)

# create x0, the starting state, the first 14 elements of the flattened xu_traj
x0 = h_xu_traj[:14]

state_differential_model = crocoddyl.StateMultibody(robot_model)

# Initialize matrices Q and R for state and control tracking cost
Q_COST = 0.1
R_COST = 0.0001
# set Q to a column vector of length state_differential_model.nx and value 0.1

Q = np.full((state_differential_model.nx, 1), Q_COST)
R = np.full((NUM_CONTROLS, 1), R_COST)

# EE cost multiplier is hardcoded to 0.5
COST_MATRIX_EE = np.full((3, 1), 0.5)

# 1. Update desired state and control sequences based on the current state
# x_des, u_des = get_desired_trajectory_slice(x0, N)  # Implement this function based on your application
# for now only look at the first 32 points
target_traj_x = xu_traj2d[0:KNOT_POINTS, 0:14]
target_traj_u = xu_traj2d[0:KNOT_POINTS, 14:]

target_ee_traj = eePos_traj2d[0:KNOT_POINTS]

# 2. Update the cost models for the N-steps lookahead
actuation = crocoddyl.ActuationModelFull(state_differential_model)
runningCostModel = crocoddyl.CostModelSum(state_differential_model)
terminalCostModel = crocoddyl.CostModelSum(state_differential_model)

for t in range(KNOT_POINTS):
    x_ref_t = target_traj_x[t]
    u_ref_t = target_traj_u[t]
    ee_pos_ref_t = target_ee_traj[t]
    
    # per https://gepettoweb.laas.fr/doc/loco-3d/crocoddyl/master/doxygen-html/classcrocoddyl_1_1CostModelResidualTpl.html#details
    # ActivationModelQuadTpl is used by default
    # if we didn't care about Q, this would be fine, but I want to be able to set Q
    # So for this we need the ActivationModelWeightedQuad
    # https://gepettoweb.laas.fr/doc/loco-3d/crocoddyl/master/doxygen-html/classcrocoddyl_1_1ActivationModelWeightedQuadTpl.html
    activation_state = crocoddyl.ActivationModelWeightedQuad(Q)
    state_error_residual = crocoddyl.ResidualModelState(state_differential_model, x_ref_t)
    state_tracking_cost = crocoddyl.CostModelResidual(state_differential_model, activation_state, state_error_residual)

    # don't actually add this to running cost for now, we don't include state offset in the loss function in gpu pcg tracking
    # runningCostModel.addCost("stateTrack" + str(t), state_tracking_cost, 1.)
    
    activation_ee = crocoddyl.ActivationModelWeightedQuad(COST_MATRIX_EE)
    goalTrackingCostXYZ = crocoddyl.CostModelResidual(
        state_differential_model,
        activation_ee,
        crocoddyl.ResidualModelFrameTranslation(
            state_differential_model, joint_7_index, ee_pos_ref_t[0:3]
        ),
    )
    runningCostModel.addCost("goalTrackXYZ" + str(t), goalTrackingCostXYZ, 1.)

    # add control tracking cost if we are not at the last knot point
    if t < KNOT_POINTS - 1:
        activation_control = crocoddyl.ActivationModelWeightedQuad(R)
        control_error_residual = crocoddyl.ResidualModelControl(state_differential_model, u_ref_t)
        control_tracking_cost = crocoddyl.CostModelResidual(state_differential_model, activation_control, control_error_residual)
        runningCostModel.addCost("ctrlTrack" + str(t), control_tracking_cost, 0.001)

x_ref_t = target_traj_x[-1]
u_ref_t = np.zeros(NUM_CONTROLS)

# terminalCostModel.addCost("stateTrack", state_tracking_cost, 10000)


running_model = crocoddyl.IntegratedActionModelEuler(
crocoddyl.DifferentialActionModelFreeFwdDynamics(
    state_differential_model, actuation, runningCostModel),
DT)

terminal_model = crocoddyl.IntegratedActionModelEuler(
crocoddyl.DifferentialActionModelFreeFwdDynamics(
    state_differential_model, actuation, terminalCostModel)
)

problem = crocoddyl.ShootingProblem(x0, [running_model] * KNOT_POINTS, terminal_model)

# Creating the DDP solver for this OC problem, defining a logger
ddp = crocoddyl.SolverDDP(problem)
ddp.setCallbacks([crocoddyl.CallbackVerbose()])

# time how long the ddp solve takes
import time
time_limit = 0.01
start_time = time.time()
# Solving it with the DDP algorithm
ddp.solve()
end_time = time.time()
print("DDP solve time: ", end_time - start_time)

# Visualizing the solution in gepetto-viewer
display.displayFromSolver(ddp)

robot_data = robot_model.createData()
xT = ddp.xs[-1]
pinocchio.forwardKinematics(robot_model, robot_data, xT[: state_differential_model.nq])
pinocchio.updateFramePlacements(robot_model, robot_data)

print("initial x")
print(x0)
# print x_ref_t
print("x_ref_t")
print(x_ref_t)
# print final x
print("And the final x:")
print(xT)

# Actual tracking stuff, just going to try a single solve for now
# 4. Apply the first control to your system
# u0 = ddp.us[0]
# x0 = apply_control_to_system(u0)  # Implement this function based on your application

# 5. Measure the new state (here it's returned by apply_control_to_system)

# 6. Loop repeats
