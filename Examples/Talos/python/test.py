import pybullet as p
import pybullet_data as pd
import matplotlib.pyplot as plt
import time
import numpy as np
import pinocchio as pin
from pathlib import Path
import scipy.io

### settings
timeStep = 0.1

### read data
# Use the full-trajectories_forward_0.0.txt file
urdf_filename =  "../../../Robots/talos/talos_reduced_armfixed_floatingbase.urdf"
model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

nq = model.nq
nv = model.nv
nu = nv - 6

# Read the trajectories from full-trajectories_forward_0.0.txt
trajectories = np.loadtxt('../data/full-trajectories_forward_0.0.txt')

# Assuming the file contains joint configurations (q) at each timestep
# Adjust ts_raptor based on the length of trajectories
ts_raptor = np.linspace(0, 0.8, len(trajectories)) 
xs_raptor = np.zeros((len(ts_raptor), nq + nv)) 
us_raptor = np.zeros((len(ts_raptor), nu))

for i, states in enumerate(trajectories):
    q = states[:nv]  # Assuming the first nv values are the joint positions
    v = states[nv:(2*nv)]  # Assuming the next nv values are the joint velocities
    u = states[(2*nv):]  # The rest are the control inputs (torques or forces)

    xs_raptor[i, :nq] = q
    xs_raptor[i, nq:] = v
    us_raptor[i, :] = u

# Extract position data
pos_sim = xs_raptor[:, :nq]

# Connect to PyBullet simulator
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

# Load the robot URDF model
robot = p.loadURDF("../../../Robots/talos/talos_reduced_armfixed.urdf", useFixedBase=False)

# Start the simulation
p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)
num_joints = p.getNumJoints(robot)

# Run the simulation loop
for tid in range(0, data.shape[1]):
    base_xyz = data[0:3, tid]
    base_rpy = data[3:6, tid]
    base_quat = p.getQuaternionFromEuler(base_rpy)
    pos = data[6:18, tid]

    # Update the robot's base position and orientation
    p.resetBasePositionAndOrientation(robot, base_xyz, base_quat)
    
    # Reset joint states based on the trajectory data
    id = 0
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_type = joint_info[2]
        if joint_type == p.JOINT_FIXED:
            p.resetJointState(robot, i, targetValue=0)
        else:
            p.resetJointState(robot, i, targetValue=pos[id])
            id += 1
    
    # Step the simulation forward
    p.stepSimulation()
    time.sleep(1e-2)
    
# Wait for user input before disconnecting
input("Press Enter to continue...")

# Disconnect from PyBullet
p.disconnect()
