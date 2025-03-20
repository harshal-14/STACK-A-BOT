import torch
import cv2
import numpy as np
from torchvision.transforms import Compose, Normalize, ToTensor

# Load MiDaS model
model_type = "DPT_Large"  # Use "DPT_Hybrid" or "MiDaS_small" if needed
model = torch.hub.load("intel-isl/MiDaS", model_type, trust_repo=True)
transform = torch.hub.load("intel-isl/MiDaS", "transforms").dpt_transform

# Input image
image_path = r"C:/Users/dell/Downloads/robotic-grasping-master/robotic-grasping-master/pcd0199r.png"
image = cv2.imread(image_path)

if image is None:
    raise ValueError(f"Failed to load image from {image_path}")

image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
print("Input image shape:", image.shape)

# Apply transformation
input_batch = transform(image).unsqueeze(0)  # Should produce [1, 3, height, width]
print("Input batch shape before reshape:", input_batch.shape)

# Check for extra dimensions and reshape if needed
if len(input_batch.shape) == 5:
    input_batch = input_batch.squeeze(1)  # Remove the extra dimension
print("Input batch shape after reshape:", input_batch.shape)

# Depth estimation
with torch.no_grad():
    prediction = model(input_batch)  # Pass the corrected input batch
    depth_map = torch.nn.functional.interpolate(
        prediction.unsqueeze(1),
        size=image.shape[:2],
        mode="bicubic",
        align_corners=False,
    ).squeeze().cpu().numpy()

# Normalize depth map for visualization
depth_map = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())

# Save depth map to a file
output_path = r"C:/Users/dell/Downloads/robotic-grasping-master/robotic-grasping-master/pcd0199r.tiff"
cv2.imwrite(output_path, (depth_map * 255).astype(np.uint8))

# Display depth map (optional)
cv2.imshow("Depth Map", depth_map)
cv2.waitKey(0)