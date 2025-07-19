import pybullet as p
import pybullet_data as pd
import time
import numpy as np

# 设置仿真时间步长
timeStep = 0.12  # 30FPS, 1/30 ≈ 0.033s

# 读取轨迹数据（仅包含 q）
trajectories = np.loadtxt('../data/test8-log.txt') 
# trajectories = np.loadtxt('../data/full-trajectory-h1-forward.txt')  # 直接读取轨迹数据
# trajectories = np.loadtxt('../data/final.txt')  # 直接读取轨迹数据
num_time_steps, nq = trajectories.shape  

# 连接到 PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
# # 设置摄像机视角
# p.resetDebugVisualizerCamera(
#     cameraDistance=3.0,               # 2 米远
#     cameraYaw=135,                     # 从 45° 角观察
#     cameraPitch=-30,                  # 俯视 30° 角
#     cameraTargetPosition=[0, 0, 0]     # 目标点为世界坐标系原点
# )


# 加载机器人模型（Talos）
robot = p.loadURDF("../../../Robots/Unitree/h1_2_12dof_floatingbase.urdf", useFixedBase=False)

# 设置重力
p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)

# 获取机器人关节数
num_joints = p.getNumJoints(robot)


# 运行仿真
for t in range(num_time_steps):
    q = trajectories[t]  
    
    joint_id = 0
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_type = joint_info[2]
        
        if joint_type == p.JOINT_FIXED:
            p.resetJointState(robot, i, targetValue=0)
        else:
            p.resetJointState(robot, i, targetValue=q[joint_id])  
            joint_id += 1 
    
    # 执行仿真
    p.stepSimulation()
    time.sleep(timeStep)  

# 结束仿真
input("Press Enter to exit...")
p.disconnect()
