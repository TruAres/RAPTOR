import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import torch
import pathlib
import os

def fanuc_rgb_renderer(q):
    """
    Render RGB images of a robot in different joint configurations.
    :param q: The joint angles of the robot in radians (B,T,Da)
    :return: The RGB images of the robot (B,T,C,H,W)
    """
    # Setup the connection to the physics simulation.
    physicsClient = p.connect(p.DIRECT) # Use p.GUI to see the simulation window

    # Add the path to the pybullet_data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the plane and a robot URDF (adjust the path to your URDF)
    urdf_path = os.path.join(pathlib.Path(__file__).parent.absolute(), "fanuc_lrmate_200id/fanuc.urdf")
    robot = p.loadURDF(urdf_path, useFixedBase=True)

    # Set the camera parameters
    camera_distance = 40
    camera_yaw = 45
    camera_pitch = -20
    camera_target_position = [2, 0, 2] # Adjust to target the robot

    B, T, _ = q.shape
    img_tensor = np.zeros((B, T, 3, 128, 128), dtype=np.uint8)

    for b in range(B):
        for t in range(T):
        # Set joint angles; replace 'joint_angles' with your actual joint angle values and 'joint_indices' with their respective indices
            joint_indices = [0, 1, 2, 3, 4, 5] # Example joint indices
            joint_angles = np.deg2rad(q[b, t].detach().cpu().numpy()) # Example joint angles in radians

        # Apply joint angles
        for index, angle in zip(joint_indices, joint_angles):
            p.resetJointState(robot, index, angle)

        # Get the camera image
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
        width=640,
        height=640,
        viewMatrix=p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=camera_target_position,
        distance=camera_distance,
        yaw=camera_yaw,
        pitch=camera_pitch,
        roll=0,
        upAxisIndex=2),
        projectionMatrix=p.computeProjectionMatrixFOV(
        fov=20,
        aspect=1,
        nearVal=0.1,
        farVal=500.0))

        # Convert the RGB image to a format suitable for Matplotlib
        rgb_array = np.array(rgbImg, dtype=np.uint8)
        rgb_array = rgb_array[:, :, :3] # Remove alpha channel

        # Resize the image
        img_resized = Image.fromarray(rgb_array).resize((128, 128))

        # Save the image to the img_tensor
        img_tensor[b, t] = np.array(img_resized).transpose(2, 0, 1)

        # move to cuda device
        img_tensor = torch.from_numpy(img_tensor).to('cuda:0')

        # Disconnect the simulation
        p.disconnect()

    return img_tensor

if __name__ == "__main__":
    # Example usage
    q = np.random.rand(2, 3, 6) # Random joint angles
    q = np.array([[0, 0, 0, 0, -90, 0],
    [0, 10, 15, 0, -90, 0],
    [0, 20, 30, 0, -90, 0],
    [0, 30, 45, 0, -90, 0],
    [0, 40, 60, 0, -90, 0],
    [0, 50, 75, 0, -90, 0]])
    # reshape q from (6,6) to (2,3,6)
    q = q.reshape(2, 3, 6)
    q = torch.from_numpy(q).to('cuda:0') # Move q to the cuda device

    img_tensor = fanuc_rgb_renderer(q)
    img_tensor = img_tensor.cpu().numpy() # Move img_tensor from cuda device to CPU
    print(img_tensor.shape) # Output: (2, 3, 3, 128, 128)
    print(img_tensor.dtype) # Output: uint8

    # Plot the images
    B, T, C, H, W = img_tensor.shape
    fig, axes = plt.subplots(B, T)
    for b in range(B):
        for t in range(T):
            axes[b, t].imshow(img_tensor[b, t].transpose(1, 2, 0))
            axes[b, t].axis('off')
    plt.show()

# # save the images
# save_dir = pathlib.Path("tmp")
# save_dir.mkdir(parents=True, exist_ok=True)
# for b in range(B):
# for t in range(T):
# img = Image.fromarray(img_tensor[b, t].transpose(1, 2, 0))
# img.save(save_dir / f"image_{b}_{t}.png")

