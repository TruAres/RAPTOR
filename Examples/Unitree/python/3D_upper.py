import pybullet as p
import pybullet_data as pd
import time
import numpy as np
import matplotlib.pyplot as plt

# 设置仿真时间步长
timeStep = 0.08  # 12.5FPS (约0.08秒/帧)

# 读取关节位置数据
try:
    joint_positions = np.loadtxt('../data/all_joint_positions_with_upper_body.txt')
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

# 设置摄像机视角
p.resetDebugVisualizerCamera(
    cameraDistance=3.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.8]
)

# 创建颜色映射
colors = plt.cm.viridis(np.linspace(0, 1, num_joints))

# 为每个关节创建可视化小球
spheres = []
for i in range(num_joints):
    sphere = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.02,
        rgbaColor=[colors[i][0], colors[i][1], colors[i][2], 1]
    )
    spheres.append(sphere)

# 创建位置标记物体
joint_markers = [p.createMultiBody(baseVisualShapeIndex=spheres[i]) for i in range(num_joints)]

# 运行轨迹可视化
for t in range(num_time_steps):
    # 获取当前帧所有关节位置
    frame_data = joint_positions[t]

    # 更新每个关节标记的位置
    for i in range(num_joints):
        start_idx = i * 3
        x, y, z = frame_data[start_idx], frame_data[start_idx + 1], frame_data[start_idx + 2]

        # 重置标记位置
        p.resetBasePositionAndOrientation(
            joint_markers[i],
            [x, y, z],
            [0, 0, 0, 1]  # 无旋转
        )

    # 添加延时以匹配实际运动速度
    p.stepSimulation()
    time.sleep(timeStep)

# 结束仿真
input("Press Enter to exit...")
p.disconnect(physicsClient)