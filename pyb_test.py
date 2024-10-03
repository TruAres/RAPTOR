import pybullet as p
import pybullet_data
import time

# 以 GUI 模式连接到 PyBullet
p.connect(p.GUI)

# 设置额外的搜索路径以找到 URDF 文件
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 重置模拟以确保环境干净
p.resetSimulation()

# 加载地面和机器人 URDF 文件
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# 设置重力
p.setGravity(0, 0, -9.81)

# 运行模拟，执行一定数量的步数
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

# 断开与物理引擎的连接
p.disconnect()
