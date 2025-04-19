import os
import cv2
import numpy as np
import pybullet as p
import pybullet_data
import time

from SimCamera import SimCamera  # Adjust import if needed
#from STACK_A_BOT.SimCamera import SimCamera
#from STACK_A_BOT.Components.SimCamera import SimCamera
#from SimCamera import Sim
# === Create output directory ===
output_dir = "dust3r_input_images"
os.makedirs(output_dir, exist_ok=True)

# === Initialize PyBullet environment ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
p.loadURDF("cube.urdf", basePosition=[0, 0, 0])  # Or your own URDF

# === Define different viewpoints ===
camera_views = [
    ([1, 1, 1], [0, 0, 0]),
    ([1, -1, 1], [0, 0, 0]),
    ([-1, 1, 1], [0, 0, 0]),
    ([-1, -1, 1], [0, 0, 0])
]

# === Use SimCamera to capture images ===
camera = SimCamera()

for idx, (pos, target) in enumerate(camera_views):
    camera.connect(camera_pos=pos, camera_target=target)
    rgb = camera.get_RGB_image()  # (3, H, W)

    # Convert CHW to HWC and scale to uint8
    rgb_hwc = (np.transpose(rgb, (1, 2, 0)) * 255).astype(np.uint8)
    rgb_bgr = cv2.cvtColor(rgb_hwc, cv2.COLOR_RGB2BGR)

    save_path = os.path.join(output_dir, f"view_{idx+1}.png")
    cv2.imwrite(save_path, rgb_bgr)
    print(f"[✓] Saved {save_path}")

    time.sleep(0.3)

p.disconnect()

#load in duster
#images = load_images([
#    "dust3r_input_images/view_1.png",
#    ...
#], size=512)ls
