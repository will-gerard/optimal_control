import pinocchio
from sys import argv
from os.path import dirname, join, abspath

import numpy as np

urdf_filename = "/home/a2rlab/Will/code/optimal_control/iiwa.urdf"

# Load the urdf model
model    = pinocchio.buildModelFromUrdf(urdf_filename)
print('model name: ' + model.name)
# Create data required by the algorithms
data     = model.createData()
# Sample a random configuration
# q        = np.zeros(model.nq)
q        = pinocchio.randomConfiguration(model)

# add some random noise to each element of q
q = q + np.random.normal(0, 0.5, model.nq)

print('q: %s' % q.T)
# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model,data,q)
# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .4f} {: .4f} {: .4f}"
          .format( name, *oMi.translation.T.flat )))
    
# get the id of iiwa_joint_7 and print out the joint placement
joint_id = model.getJointId("iiwa_joint_7")
print('joint_id: %s' % joint_id)
print('joint placement: %s' % data.oMi[joint_id].translation.T)

# get the id of the end effector frame and print out the placement
# frame_id = model.getFrameId("iiwa_frame_ee")
# print('frame_id: %s' % frame_id)
# print('frame placement: %s' % data.oMf[frame_id].translation.T)

# print the name and placement of every frame
# for frame_id in range(model.nframes):
#     print('frame_id: %s' % frame_id)
#     print('frame name: %s' % model.frames[frame_id].name)
#     print('frame placement: %s' % data.oMf[frame_id].translation.T)