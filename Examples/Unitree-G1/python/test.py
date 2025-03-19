import pybullet as p
import pybullet_data as pd
import time
import numpy as np


timeStep = 0.1  


trajectories = np.loadtxt('../data/full-trajectory-g1-forward.txt') 
num_time_steps, nq = trajectories.shape  


p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())


robot = p.loadURDF("../../../Robots/unitree-g1/g1_12dof_floatingbase.urdf", useFixedBase=False)

p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)


num_joints = p.getNumJoints(robot)

# 运行仿真
for t in range(num_time_steps):
    q = trajectories[t]  
    
    joint_id = 0
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_type = joint_info[2]
        
        if joint_type == p.JOINT_FIXED:
            continue  
        else:
            p.resetJointState(robot, i, targetValue=q[joint_id]) 
            joint_id += 1  

    p.stepSimulation()
    time.sleep(timeStep)  

# 结束仿真
input("Press Enter to exit...")
p.disconnect()
