"""
Script to check the SimCamera class and its functions
"""

import os
import sys
import numpy as np
import cv2
import pybullet as p
import pybullet_data
import time

# Ensure local import works
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from ..Components.Sim.SimCamera import SimCamera  # Adjust path if needed


def setup_pybullet_env():
    """Initialize the PyBullet simulation environment."""
    #p.connect(p.GUI)
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    #p.loadURDF("urdf/box.urdf", basePosition=[0.5, 0, 0])
    #p.loadURDF("../World/models/box0.urdf", basePosition=[0.5, 0, 0])
    p.loadURDF("/home/raval/cap_ws/src/STACK_A_BOT/World/models/box0.urdf", basePosition=[0.5, 0, 0])


    #/home/raval/cap_ws/src/STACK_A_BOT/World/models/box0.urdf
    #/home/raval/cap_ws/src/STACK_A_BOT/Tests/test_sim_camera.py
    

    for _ in range(50):
        p.stepSimulation()
        time.sleep(1. / 240.)


def main():
    print("Setting up PyBullet simulation...")
    setup_pybullet_env()

    print("Initializing SimCamera...")
    camera = SimCamera()
    if camera.connect(camera_pos=[1, 1, 1], camera_target=[0, 0, 0]) != 0:
        print("Failed to connect SimCamera. Exiting.")
        return 1

    try:
        print("Testing RGB image capture...")
        rgb_frame = camera.get_RGB_image()
        print(f"RGB frame shape: {rgb_frame.shape}")
        rgb_display = np.transpose(rgb_frame, (1, 2, 0))  # CHW to HWC
        rgb_display = (rgb_display * 255).astype(np.uint8)
        cv2.imshow("RGB Image", rgb_display)
        cv2.waitKey(1000)

        print("Testing depth image capture...")
        depth_frame = camera.get_depth_data()
        print(f"Depth frame shape: {depth_frame.shape}")
        depth_display = depth_frame[0]
        depth_colormap = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1000)

        print("Testing mono image capture...")
        left_frame, right_frame = camera.get_mono_images()
        print(f"Left frame shape: {left_frame.shape}")
        print(f"Right frame shape: {right_frame.shape}")
        left_display = (left_frame[0] * 255).astype(np.uint8)
        right_display = (right_frame[0] * 255).astype(np.uint8)
        cv2.imshow("Left Mono", left_display)
        cv2.imshow("Right Mono", right_display)

        print("Press any key to close windows and finish testing...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        print("\nSimCamera test completed successfully!")

    except Exception as e:
        print(f"Error during SimCamera test: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        print("Disconnecting from SimCamera and shutting down PyBullet...")
        camera.disconnect()
        p.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
