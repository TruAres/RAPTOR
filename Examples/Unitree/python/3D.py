# import pybullet as p
# import pybullet_data as pd
# import time
# import numpy as np
# import matplotlib.pyplot as plt

# # 设置仿真时间步长
# timeStep = 0.08  # 12.5FPS (约0.08秒/帧)

# # 读取关节位置数据
# joint_positions = np.loadtxt('../data/all_joint_positions.txt')
# num_time_steps, dim = joint_positions.shape

# # 计算总关节数 (每关节3个坐标值)
# num_joints = dim // 3
# print(f"Loaded {num_time_steps} frames with {num_joints} joints")

# # 连接到 PyBullet
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pd.getDataPath())
# p.setGravity(0, 0, -9.81)
# p.setTimeStep(timeStep)

# # 设置摄像机视角
# p.resetDebugVisualizerCamera(
#     cameraDistance=3.0,
#     cameraYaw=45,
#     cameraPitch=-30,
#     cameraTargetPosition=[0, 0, 0.8]
# )

# # 创建颜色映射
# colors = plt.cm.viridis(np.linspace(0, 1, num_joints))

# # 为每个关节创建可视化小球
# spheres = []
# for i in range(num_joints):
#     sphere = p.createVisualShape(
#         p.GEOM_SPHERE,
#         radius=0.02,
#         rgbaColor=[colors[i][0], colors[i][1], colors[i][2], 1]
#     )
#     spheres.append(sphere)
    
# # 创建位置标记物体
# joint_markers = [p.createMultiBody(baseVisualShapeIndex=spheres[i]) for i in range(num_joints)]

# # 运行轨迹可视化
# for t in range(num_time_steps):
#     # 获取当前帧所有关节位置
#     frame_data = joint_positions[t]
    
#     # 更新每个关节标记的位置
#     for i in range(num_joints):
#         start_idx = i * 3
#         x, y, z = frame_data[start_idx], frame_data[start_idx+1], frame_data[start_idx+2]
        
#         # 重置标记位置
#         p.resetBasePositionAndOrientation(
#             joint_markers[i],
#             [x, y, z],
#             [0, 0, 0, 1]  # 无旋转
#         )
    
#     # 添加延时以匹配实际运动速度
#     p.stepSimulation()
#     time.sleep(timeStep)

# # 结束仿真
# input("Press Enter to exit...")
# p.disconnect()


import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import matplotlib.pyplot as plt

# 设置仿真时间步长
timeStep = 0.08  # 12.5FPS (约0.08秒/帧)

# 读取关节位置数据
try:
    joint_positions = np.loadtxt('../data/all_joint_positions.txt')
    num_time_steps, dim = joint_positions.shape
except Exception as e:
    print(f"Error loading data: {e}")
    exit(1)

# 计算总关节数 (每关节3个坐标值)
num_joints = dim // 3
print(f"Loaded {num_time_steps} frames with {num_joints} joints")

# 连接到 PyBullet
physicsClient = p.connect(p.GUI)  # 保存client ID以便之后断开
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(timeStep)

# 设置更好的摄像机视角
p.resetDebugVisualizerCamera(
    cameraDistance=2.5,        # 更近一点以便观察细节
    cameraYaw=45,              # 侧角度观察
    cameraPitch=-20,           # 稍微俯视
    cameraTargetPosition=[0, 0, 0.8]
)

# 可选：加载地板作为参考平面
planeId = p.loadURDF("plane.urdf", [0, 0, -0.1])

# 创建更有区分度的颜色映射
try:
    colors = plt.cm.gist_rainbow(np.linspace(0, 1, num_joints))
except:
    # 后备颜色方案
    colors = np.zeros((num_joints, 4))
    for i in range(num_joints):
        hue = i / float(num_joints)
        colors[i] = [hue, 1 - hue, 0.5 + (0.5 * hue), 1.0]

# 为每个关节创建可视化小球，使用不同的半径表示关节重要性
joint_markers = []
sphere_radii = []
for i in range(num_joints):
    # 根据关节类型设置不同半径
    if i < 6:  # 假设前6个是主要关节（基座）
        radius = 0.035
    elif i < 12:  # 中段关节
        radius = 0.025
    else:  # 末端关节
        radius = 0.015
    
    sphere = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=[colors[i][0], colors[i][1], colors[i][2], 1]
    )
    # 创建实体用于可视化
    marker_id = p.createMultiBody(
        baseMass=0,  # 无质量
        baseVisualShapeIndex=sphere,
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 1]
    )
    joint_markers.append(marker_id)
    sphere_radii.append(radius)

# 可选：添加文本标签
text_ids = []
label_positions = np.zeros((num_joints, 3))
if num_joints <= 20:  # 只有在关节数不太多时添加标签
    for i in range(num_joints):
        # 初始位置在关节上方
        label_positions[i] = [0, 0, sphere_radii[i] + 0.02]
        text_ids.append(p.addUserDebugText(
            f"J{i}",
            label_positions[i],
            textColorRGB=[colors[i][0], colors[i][1], colors[i][2]],
            parentObjectUniqueId=joint_markers[i],
            parentLinkIndex=-1
        ))

# 添加轨迹线
line_ids = []
prev_positions = None

# 运行轨迹可视化
for t in range(num_time_steps):
    # 获取当前帧所有关节位置
    frame_data = joint_positions[t]
    
    # 更新每个关节标记的位置
    current_positions = []
    for i in range(num_joints):
        start_idx = i * 3
        x, y, z = frame_data[start_idx], frame_data[start_idx+1], frame_data[start_idx+2]
        current_positions.append([x, y, z])
        
        # 重置标记位置
        p.resetBasePositionAndOrientation(
            joint_markers[i],
            [x, y, z],
            [0, 0, 0, 1]  # 无旋转
        )
    
    # 绘制轨迹线（每10帧更新一次）
    if t > 0 and (t % 10 == 0 or t == num_time_steps - 1):
        # 先移除旧线条
        for line_id in line_ids:
            p.removeUserDebugItem(line_id)
        line_ids = []
        
        # 绘制新线条（连接前10个关节）
        for i in range(min(10, num_joints)):
            if prev_positions is not None:
                line_ids.append(p.addUserDebugLine(
                    prev_positions[i],
                    current_positions[i],
                    lineColorRGB=[colors[i][0], colors[i][1], colors[i][2]],
                    lineWidth=2,
                    lifeTime=0  # 永久存在
                ))
    
    # 保存当前位置用于下一帧线条
    prev_positions = current_positions
    
    # 更新文本标签位置
    for i, text_id in enumerate(text_ids):
        # 将标签放在关节上方
        p.addUserDebugText(
            f"J{i}",
            [0, 0, sphere_radii[i] + 0.05],  # 关节上方
            textColorRGB=[colors[i][0], colors[i][1], colors[i][2]],
            replaceItemUniqueId=text_id,
            parentObjectUniqueId=joint_markers[i]
        )
    
    # 添加延时以匹配实际运动速度
    p.stepSimulation()
    time.sleep(timeStep)

# 结束仿真
input("Press Enter to exit...")
p.disconnect(physicsClient)