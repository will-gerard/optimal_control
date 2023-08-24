import numpy as np
import pinocchio

import crocoddyl
import os
print("Current Working Directory:", os.getcwd())

from robot_descriptions.loaders.pinocchio import load_robot_description

robot: pinocchio.RobotWrapper = load_robot_description("iiwa_description")
robot_model = robot.model

DT = 1e-3
KNOT_POINTS = 512
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
Q = np.eye(state_differential_model.nx) * 0.1  # 1 is the weight for the state tracking
R = np.eye(NUM_CONTROLS) * 0.0001  # 1 is the weight for the control tracking

# 1. Update desired state and control sequences based on the current state
# x_des, u_des = get_desired_trajectory_slice(x0, N)  # Implement this function based on your application
# for now only look at the first 32 points
target_traj_x = xu_traj2d[0:KNOT_POINTS, 0:14]
target_traj_u = xu_traj2d[0:KNOT_POINTS, 14:]


# 2. Update the cost models for the N-steps lookahead
actuation = crocoddyl.ActuationModelFull(state_differential_model)
runningCostModel = crocoddyl.CostModelSum(state_differential_model)
terminalCostModel = crocoddyl.CostModelSum(state_differential_model)
# at first I thought I had to loop over every know and add the cost for that point, now I think 
# I just need to add the cost for the whole trajectory with the state differential model
# for t in range(KNOT_POINTS):
#     x_ref_t = target_traj_x[t]
#     u_ref_t = target_traj_u[t]
    
#     # per https://gepettoweb.laas.fr/doc/loco-3d/crocoddyl/master/doxygen-html/classcrocoddyl_1_1CostModelResidualTpl.html#details
#     # ActivationModelQuadTpl is used by default, so not passing an activation model to this cost residual
#     state_tracking_cost = crocoddyl.CostModelResidual(state_differential_model, 
#                                                     crocoddyl.ResidualModelState(state_differential_model, x_ref_t))
#     control_tracking_cost = crocoddyl.CostModelResidual(state_differential_model,
#                                                     crocoddyl.ResidualModelControl(state_differential_model, u_ref_t))

#     runningCostModel.addCost("stateTrack" + str(t), state_tracking_cost, 1.)
#     runningCostModel.addCost("ctrlTrack" + str(t), control_tracking_cost, 1.)
x_ref_t = target_traj_x[-1]
u_ref_t = np.zeros(NUM_CONTROLS)

# print x_ref_t
print("x_ref_t")
print(x_ref_t)

# per https://gepettoweb.laas.fr/doc/loco-3d/crocoddyl/master/doxygen-html/classcrocoddyl_1_1CostModelResidualTpl.html#details
# ActivationModelQuadTpl is used by default, so not passing an activation model to this cost residual
state_tracking_cost = crocoddyl.CostModelResidual(state_differential_model, 
                                                crocoddyl.ResidualModelState(state_differential_model, x_ref_t))
# control_tracking_cost = crocoddyl.CostModelResidual(state_differential_model,
#                                                 crocoddyl.ResidualModelControl(state_differential_model, u_ref_t))

runningCostModel.addCost("stateTrack", state_tracking_cost, 1.)
#runningCostModel.addCost("ctrlTrack", control_tracking_cost, 1.)

terminalCostModel.addCost("stateTrack", state_tracking_cost, 100)


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

# Solving it with the DDP algorithm
ddp.solve()

# Visualizing the solution in gepetto-viewer
display.displayFromSolver(ddp)

robot_data = robot_model.createData()
xT = ddp.xs[-1]
pinocchio.forwardKinematics(robot_model, robot_data, xT[: state_differential_model.nq])
pinocchio.updateFramePlacements(robot_model, robot_data)

# print final x
print(xT)

# Actual tracking stuff, just going to try a single solve for now
# 4. Apply the first control to your system
# u0 = ddp.us[0]
# x0 = apply_control_to_system(u0)  # Implement this function based on your application

# 5. Measure the new state (here it's returned by apply_control_to_system)

# 6. Loop repeats
