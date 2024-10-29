# import pybullet as p
# import pybullet_data as pd
# import matplotlib.pyplot as plt
# import time
# import numpy as np

# ### settings
# timeStep = 0.1

# ### read data
# data = np.loadtxt("../data/trajectory-talos-simulation.txt")

# ### connect to simulator
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pd.getDataPath())

# # Load a simple plane
# # plane_id = p.loadURDF("plane.urdf")
# robot = p.loadURDF("../../../Robots/talos/talos_reduced_armfixed.urdf", useFixedBase=False)

# # Start the simulation
# p.setGravity(0, 0, -9.81)
# p.setTimeStep(timeStep)
# num_joints = p.getNumJoints(robot)

# # input("Press Enter to continue...")

# for tid in range(0, data.shape[1]):
#     base_xyz = data[0:3, tid]
#     base_rpy = data[3:6, tid]
#     base_quat = p.getQuaternionFromEuler(base_rpy)
#     pos = data[6:18, tid]
    
#     p.resetBasePositionAndOrientation(robot, base_xyz, base_quat)
    
#     id = 0
#     for i in range(num_joints):
#         joint_info = p.getJointInfo(robot, i)
#         joint_type = joint_info[2]
#         if joint_type == p.JOINT_FIXED:
#             p.resetJointState(robot, i, targetValue=0)
#         else:
#             p.resetJointState(robot, i, targetValue=pos[id])
#             id += 1
    
#     p.stepSimulation()
#     time.sleep(1e-2)
    
# input("Press Enter to continue...")

# # Disconnect from PyBullet
# p.disconnect()

# import pybullet as p
# import pybullet_data as pd
# import matplotlib.pyplot as plt
# import time
# import numpy as np

# ### settings
# timeStep = 0.005

# ### read data
# data = np.loadtxt("../data/trajectory-talos-simulation.txt")

# ### connect to simulator
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pd.getDataPath())

# # Load the robot
# robot = p.loadURDF("../../../Robots/talos/talos_reduced_armfixed.urdf", useFixedBase=False)

# # Start the simulation
# p.setGravity(0, 0, -9.81)
# p.setTimeStep(timeStep)
# num_joints = p.getNumJoints(robot)

# # 寻找右足脚底板的链接索引
# right_foot_link_name = "right_sole_link"
# right_foot_link_index = None

# for i in range(num_joints):
#     joint_info = p.getJointInfo(robot, i)
#     if joint_info[12].decode('utf-8') == right_foot_link_name:
#         right_foot_link_index = i
#         break

# if right_foot_link_index is None:
#     print("未找到右足脚底板链接，检查链接名称是否正确。")
#     p.disconnect()
#     exit()

# # 初始化一个列表来存储右足脚底板的轨迹点
# trajectory_points = []

# # input("Press Enter to continue...")

# # 仿真循环
# for tid in range(0, data.shape[1]):
#     base_xyz = data[0:3, tid]
#     base_rpy = data[3:6, tid]
#     base_quat = p.getQuaternionFromEuler(base_rpy)
#     pos = data[6:18, tid]
    
#     # 重置机器人的姿态
#     p.resetBasePositionAndOrientation(robot, base_xyz, base_quat)
    
#     id = 0
#     for i in range(num_joints):
#         joint_info = p.getJointInfo(robot, i)
#         joint_type = joint_info[2]
#         if joint_type == p.JOINT_FIXED:
#             p.resetJointState(robot, i, targetValue=0)
#         else:
#             p.resetJointState(robot, i, targetValue=pos[id])
#             id += 1
    
#     # 获取右足脚底板的位置
#     link_state = p.getLinkState(robot, right_foot_link_index)
#     foot_position = link_state[0]  # 位置是返回值的第一个元素
#     # foot_orientation = link_state[1]  # 如果需要方向，也可以获取
    
#     # 将当前点添加到轨迹点列表中
#     trajectory_points.append(foot_position)
    
#     # 如果有前一个点，则绘制从前一点到当前点的线
#     if len(trajectory_points) > 1:
#         start_point = trajectory_points[-2]
#         end_point = trajectory_points[-1]
#         # 添加一条红色的调试线，线宽为2，持续时间为0（持续存在）
#         p.addUserDebugLine(start_point, end_point, [1, 0, 0], lineWidth=20, lifeTime=0)
    
#     p.stepSimulation()
#     time.sleep(timeStep)

# input("Press Enter to continue...")

# # Disconnect from PyBullet
# p.disconnect()

import pybullet as p
import pybullet_data as pd
import matplotlib.pyplot as plt
import time
import numpy as np

### settings
timeStep = 0.005

### read data
data = np.loadtxt("../data/trajectory-talos-simulation.txt")

### connect to simulator
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())

# Load the robot
robot = p.loadURDF("../../../Robots/talos/talos_reduced_armfixed.urdf", useFixedBase=False)

# Start the simulation
p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)
num_joints = p.getNumJoints(robot)

# 寻找右足脚底板的链接索引
right_foot_link_name = "right_sole_link"
right_foot_link_index = None

for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    if joint_info[12].decode('utf-8') == right_foot_link_name:
        right_foot_link_index = i
        break

if right_foot_link_index is None:
    print("未找到右足脚底板链接，检查链接名称是否正确。")
    p.disconnect()
    exit()

# 寻找左手的链接索引
left_hand_link_name = "arm_right_7_link"
left_hand_link_index = None

for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    if joint_info[12].decode('utf-8') == left_hand_link_name:
        left_hand_link_index = i
        break

if left_hand_link_index is None:
    print("未找到左手链接，检查链接名称是否正确。")
    p.disconnect()
    exit()

# 初始化列表来存储右足和左手的轨迹点
foot_trajectory_points = []
hand_trajectory_points = []

# 仿真循环
for tid in range(0, data.shape[1]):
    base_xyz = data[0:3, tid]
    base_rpy = data[3:6, tid]
    base_quat = p.getQuaternionFromEuler(base_rpy)
    pos = data[6:18, tid]
    
    # 重置机器人的姿态
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
    
    # 获取右足脚底板的位置
    foot_link_state = p.getLinkState(robot, right_foot_link_index)
    foot_position = foot_link_state[0]  # 位置是返回值的第一个元素
    
    # 获取左手的位置
    hand_link_state = p.getLinkState(robot, left_hand_link_index)
    hand_position = hand_link_state[0]
    
    # 将当前点添加到轨迹点列表中
    foot_trajectory_points.append(foot_position)
    hand_trajectory_points.append(hand_position)
    

    # 绘制右足的轨迹线（红色）
    if (tid % 80 == 0) and (len(foot_trajectory_points) > 1) :
        start_point = foot_trajectory_points[-80]
        end_point = foot_trajectory_points[-1]
        p.addUserDebugLine(start_point, end_point, [1, 0, 0], lineWidth=20, lifeTime=0)
    
    # 绘制左手的轨迹线（蓝色）
    if (tid % 80 == 0) and (len(hand_trajectory_points) > 1):
        start_point = hand_trajectory_points[-80]
        end_point = hand_trajectory_points[-1]
        p.addUserDebugLine(start_point, end_point, [0, 0, 1], lineWidth=20, lifeTime=0)
    
    p.stepSimulation()
    time.sleep(1e-2)

input("Press Enter to continue...")

# Disconnect from PyBullet
p.disconnect()
