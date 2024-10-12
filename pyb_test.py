import numpy as np
import pickle
import matplotlib.pyplot as plt
import build.lib.KinovaIKMotion_nanobind as ik_motion_nanobind
import pybullet as p
import pinocchio as pin
import time
import math

# urdf_filename = "robots/kinova/kinova.urdf"
urdf_filename = "Robots/kinova-gen3/kinova.urdf"

p.connect(p.GUI)
robot = p.loadURDF(urdf_filename, useFixedBase=True)
num_joints = p.getNumJoints(robot)

model = pin.buildModelFromUrdf(urdf_filename)
data = model.createData()

ik_motion_solver = ik_motion_nanobind.KinovaIKMotionPybindWrapper(urdf_filename, False)

q0 = np.array([0., 0.76179939, 3.14159265, -1.5689280271795864, 0., -0.75, 0])

ee_name = 'end_effector_link'
ee_id = model.getFrameId(ee_name)
pin.forwardKinematics(model, data, q0)
pin.updateFramePlacements(model, data)

frame_config = data.oMf[ee_id]
print(frame_config)

# set up desired end-effector transforms
# This motion just moves the end-effector along the x-axis for 20 cm
# N = 10
# N = 35
# step_size = 0.02

# desiredTransforms = np.zeros((N, 12))

# for pid in range(N):
#     # desiredTranslation = np.array([0.45 + pid * step_size, 0, 0.2])
#     if pid <=  (2 * N) // 7:
#         desiredTranslation = np.array([0.45 + pid * step_size, 0, 0.2])
#     elif pid <= (3 * N) // 7:
#         desiredTranslation = np.array([0.45 + ((2 * N) // 7) * step_size - (pid - 2 * N // 7) * step_size, 0, 0.2])
#     elif pid <= (4 * N) // 7:
#         desiredTranslation = np.array([0.45 + (N // 7) * step_size, 0 - (pid - 3 * N // 7) * step_size, 0.2])
#     elif pid <= (5 * N) // 7:
#         desiredTranslation = np.array([0.45 + (N // 7) * step_size + (pid - 4 * N //7) * step_size, 0 - N // 7 * step_size, 0.2])
#     else:
#         desiredTranslation = np.array([0.45 + (2 * N // 7) * step_size - (pid - 5 * N //7) * step_size, 0 - N // 7 * step_size, 0.2])
#     # if pid <=  N // 2:
#     #     desiredTranslation = np.array([0.45 + pid * step_size, 0, 0.2])
#     # else:
#     #     desiredTranslation = np.array([0.45 + (pid - N // 2 ) * step_size, 0.2, 0.2])
    
#     desiredRotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
#     desiredTransforms[pid] = np.concatenate([desiredTranslation, desiredRotation.flatten()])

N = 40
step_size = 0.02

desiredTransforms = np.zeros((N, 12))

# 将参数 t 分成两部分：左瓣和右瓣
N_left = N // 2
N_right = N - N_left
# 左瓣：从 t = π 到 t = 2π
t_left = np.linspace(np.pi, 2 * np.pi, N_left, endpoint=False)
# 右瓣：从 t = 2π 到 t = 3π
t_right = np.linspace(2 * np.pi, 3 * np.pi, N_right)
# 合并 t
t = np.concatenate([t_left, t_right])
# 爱心的参数方程
x = 16 * np.sin(t) ** 3
y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)

# 将 y 移动，使起点 y[0] 为 0
y_shifted = y - y[0]  # y_shifted[0] = 0

# 缩放 y_shifted 到期望的最大值
desired_y_max = 0.2  # 调整高度
y_max_shifted = y_shifted.max()
y_normalized = y_shifted / y_max_shifted  # 归一化到 [0, 1]
y_scaled = y_normalized * desired_y_max  # 缩放到 [0, desired_y_max]

# 同样处理 x
# 将 x 移动，使起点 x[0] 为 0
x_shifted = x - x[0]  # x_shifted[0] = 0
# 计算 x_shifted 的最小值和最大值
x_min_shifted = x_shifted.min()
x_max_shifted = x_shifted.max()
# 归一化 x_shifted 到 [-1, 1]
x_normalized = x_shifted / max(abs(x_min_shifted), x_max_shifted)
# 缩放 x_normalized 到期望的宽度
desired_x_range = 0.1  # 爱心的半宽度
x_scaled = x_normalized * desired_x_range

# 平移 x_scaled，使起点在 x = 0.45
x_offset = 0.45
x_scaled = x_scaled + x_offset  # x_scaled[0] = x_offset

# 填充 desiredTransforms
for pid in range(N):
    desiredTranslation = np.array([x_scaled[pid], y_scaled[pid], 0.2])

    desiredRotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    desiredTransforms[pid] = np.concatenate([desiredTranslation, desiredRotation.flatten()])


# ipopt settings
ipopt_tol = 1e-5
ipopt_constr_viol_tol = 1e-8
ipopt_obj_scaling_factor = 1.0
ipopt_max_wall_time = 0.2
ipopt_print_level = 0
ipopt_mu_strategy = "monotone"
ipopt_linear_solver = "ma86"
ipopt_gradient_check = False

# set up ik solver
ik_motion_solver.set_desired_endeffector_transforms(desiredTransforms)
ik_motion_solver.set_ipopt_parameters(ipopt_tol, \
ipopt_constr_viol_tol, \
ipopt_obj_scaling_factor, \
ipopt_max_wall_time, \
ipopt_print_level, \
ipopt_mu_strategy, \
ipopt_linear_solver,
ipopt_gradient_check)
qs, if_success = ik_motion_solver.solve(q0)


# visualize the motion
for pid, desiredTransform in enumerate(desiredTransforms):
    id = 0
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_type = joint_info[2]
        if joint_type == p.JOINT_FIXED:
            p.resetJointState(robot, i, targetValue=0)
        else:
            p.resetJointState(robot, i, targetValue=qs[id, pid])
            id += 1

    radius = 0.02 # Radius of the sphere
    mass = 0.0 # Mass of the sphere
    start_position = desiredTransform[:3] # Position of the sphere
    start_orientation = [0, 0, 0, 1] # Orientation of the sphere
    collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=radius)
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=[1, 0, 0, 1])
    sphere_id = p.createMultiBody(
    baseMass=mass,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=start_position,
    baseOrientation=start_orientation
    )

    p.stepSimulation()

    input("Press Enter to start the motion")