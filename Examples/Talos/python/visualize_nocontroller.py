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
# data = np.loadtxt("../data/trajectory-talos-simulation.txt")
urdf_filename =  "../../../Robots/talos/talos_reduced_armfixed_floatingbase.urdf"
model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

nq = model.nq
nv = model.nv
nu = nv - 6


step_length = 0.4
trajectories = np.loadtxt('../data/solution-talos-forward-' + str(step_length) + '.txt')

ts_raptor = np.linspace(0, 0.8, len(trajectories)) 
xs_raptor = np.zeros((len(ts_raptor), nq + nv)) 
us_raptor = np.zeros((len(ts_raptor), nu))

for i, states in enumerate(trajectories):
    q = states[:nv]
    v = states[nv:(2*nv)]
    u = states[(2*nv):]
    
    xs_raptor[i, :nq] = q
    xs_raptor[i, nq:] = v
    us_raptor[i, :] = u

pos_sim = xs_raptor[:, :nq]
# vel_sim = xs_raptor[:, nq:]
# us_sim = us_raptor

# e_sim = np.zeros_like(pos_sim)
# edot_sim = np.zeros_like(vel_sim)


# RF_id = model.getFrameId("right_sole_link")
# pin.forwardKinematics(model, data, xs_raptor[-1][:nq])
# pin.updateFramePlacements(model, data)
# RF_placement = data.oMf[RF_id]
# step_length_opt = RF_placement.translation[0]
# step_length_sim = step_length_opt 

# print(step_length_opt, step_length_sim)

data = pos_sim.T

### connect to simulator
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

# Load a simple plane
# plane_id = p.loadURDF("plane.urdf")
robot = p.loadURDF("../../../Robots/talos/talos_reduced_armfixed.urdf", useFixedBase=False)

# Start the simulation
p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)
num_joints = p.getNumJoints(robot)

# input("Press Enter to continue...")

for tid in range(0, data.shape[1]):
    base_xyz = data[0:3, tid]
    base_rpy = data[3:6, tid]
    base_quat = p.getQuaternionFromEuler(base_rpy)
    pos = data[6:18, tid]
    
    p.resetBasePositionAndOrientation(robot, base_xyz, base_quat)
    
    id = 0
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_type = joint_info[2]
        if joint_type == p.JOINT_FIXED:
            p.resetJointState(robot, i, targetValue=0)
        else:
            p.resetJointState(robot, i, targetValue=pos[id])
            id += 1
    
    p.stepSimulation()
    time.sleep(1e-2)
    
input("Press Enter to continue...")

# Disconnect from PyBullet
p.disconnect()