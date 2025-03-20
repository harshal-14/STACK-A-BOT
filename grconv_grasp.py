import torch.nn.functional as F
import argparse
import logging
import time
import numpy as np
import torch.utils.data
from imageio import imread      # For reading image files
import cv2                      # OpenCV library for image processing
import torch                    # PyTorch library for deep learning models
import matplotlib.pyplot as plt # For visualization
import sys

import os
absPath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(absPath)                                # Adding the path to the custom modules for grasp detection

import post_process                 # Importing post-processing function directly from the model repository
from grasp import detect_grasps     # Importing function to detect grasp points from model output (from repository)


# GraspDetectionNet class defines the flow of loading, preprocessing, and detecting grasps
class GraspDetectionNet():
    
    def __init__(self):
        # Initialize and load a pre-trained grasp detection model
        model_path = absPath + "/trained_models/GRConvNet3/epoch_30_iou_0.97"   
        self.network = torch.load(model_path, map_location='cpu')                            # Load the model to the CPU
        self.device = torch.device("cpu")                                                    # Set the device to CPU (could be GPU if available)
        self.network.to(self.device)                                                         # Move the model to the selected device

    # Load and preprocess the input RGB and depth images
    def load_and_preprocess_image(self, rgb_img, depth_img):
        og_depth = depth_img.copy()  # Keep a copy of the original depth image for later use
        depth_img_preprocessed = self.preprocess_depth_image(depth_img)  # Preprocess depth image
        return rgb_img, depth_img_preprocessed ,og_depth  # Return both the original and preprocessed depth images

    # Preprocess the depth image to prepare it for model input
    def preprocess_depth_image(self, depth_image):
        depth_copy = depth_image.copy()  
        depth_copy = np.pad(depth_copy, ((1, 1), (1, 1)), mode='constant', constant_values=np.nan)  # Pad the image with NaNs
        mask = np.isnan(depth_copy).astype(np.uint8)  # Create a mask where depth values are NaN

        
        mask_dilated = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)   # Dilation to smooth the mask edges

        depth_copy[mask_dilated == 1] = 0                                          # Replace NaN values with zeros in the depth image

        # Normalize the depth image
        max_val = np.max(np.abs(depth_copy))                                       # Find the maximum depth value for scaling
        depth_copy = depth_copy.astype(np.float32) / max_val

        depth_inpainted = cv2.inpaint(depth_copy, mask_dilated, 1, cv2.INPAINT_NS)   # Inpaint missing pixels to fix holes in the image
        
        return depth_inpainted[1:-1, 1:-1]  # Return the inpainted depth image without the padding

    # Format the RGB and depth images as input to the model
    def format_inputs(self, rgb_img, depth_img):

        # Normalize and prepare the RGB image
        rgb_img = rgb_img.astype(np.float32) / 255.0             # Normalize the pixel values to range [0, 1]
        rgb_img -= rgb_img.mean()                                # Subtract the mean for normalization
        rgb_img = rgb_img.transpose((2, 0, 1))                   # Transpose dimensions to match PyTorch format (C, H, W)

        # Normalize the depth image
        depth_img = np.clip(depth_img - np.mean(depth_img), -1, 1)  # Clip depth image to [-1, 1]
        depth_img = np.uint8(depth_img)  # Convert depth image to unsigned 8-bit integer

        # Combine the RGB and depth images as model input
        combined_input = np.vstack([np.expand_dims(depth_img, 0), rgb_img])  # Stack depth and RGB channels

        return torch.from_numpy(combined_input.astype(np.float32)).to(self.device)  # Convert to PyTorch tensor and move to device

    # Predict grasp points from the model output given an RGB and depth image
    def predict_grasp(self, rgb_img, depth_img):
        input_tensor = self.format_inputs(rgb_img, depth_img).unsqueeze(0)  # Format input and add batch dimension

        with torch.no_grad():                                               # Disable gradient computation for inference
            output = self.network.predict(input_tensor)                     # Forward pass through the model

            # Extract the four outputs from the model: grasp quality, angle, width, and cosine/sine components of angle
            quality_img = output['pos']   
            cos_angle = output['cos']
            sin_angle = output['sin']
            width_img = output['width']

        # Post-process the model output to compute grasp rectangles
        return post_process.post_process_output(quality_img, cos_angle, sin_angle, width_img)

    # Visualize the detected grasps on the RGB image
    def visualize_grasp(self, rgb_img, grasp_rectangles):
        fig, ax = plt.subplots()  
        ax.imshow(rgb_img)  

        # Plot each detected grasp rectangle on the image
        for grasp in grasp_rectangles:
            grasp.plot(ax)

        ax.set_title('Predicted Grasp Rectangle')  
        ax.axis('off') 
        plt.savefig('grasp_prediction.png')  # Save the visualization to a file
        plt.show()  
    
    # Plot the grasp metrics: quality, width, and angle maps
    def plot_metrics(self, quality, width, angle):
        # Create a figure with 3 subplots
        fig, axs = plt.subplots(1, 3, figsize=(15, 5))

        # Plot the quality map
        axs[0].imshow(quality, cmap='RdYlBu_r')
        axs[0].set_title('Quality Map')
        axs[0].axis('off')

        # Plot the angle map
        axs[2].imshow(angle, cmap='hsv_r')
        axs[2].set_title('Angle Map')
        axs[2].axis('off')

        # Plot the width map
        axs[1].imshow(width, cmap='RdYlBu_r')
        axs[1].set_title('Width Map')
        axs[1].axis('off')

        plt.tight_layout()  # Adjust layout to prevent overlapping
        plt.show()  # Display the plot

    # Main function to run the complete grasp detection pipeline
    def run_grasp_detection(self, rgb, depth):
        rgb_img, depth_img, og_depth = self.load_and_preprocess_image(rgb, depth)  # Load and preprocess images
        quality, angle, width = self.predict_grasp(rgb_img, depth_img)             # Run model to predict grasp

        grasp_rectangles = detect_grasps(quality, angle, width, no_grasps=1)  # Detect grasp points from model output

        # Extract and print grasp details
        x_cen = round(grasp_rectangles[0].center[0], 5)
        y_cen = round(grasp_rectangles[0].center[1], 5)
        w = round(grasp_rectangles[0].width, 5)
        h = round(grasp_rectangles[0].length, 5)
        theta = round(grasp_rectangles[0].angle, 5)

        print(f"Grasp Center: ({x_cen}, {y_cen}), Width: {w}, Height: {h}, Angle: {theta}")
        self.visualize_grasp(rgb_img, grasp_rectangles)  
        return [x_cen, y_cen, w, h, theta], og_depth, grasp_rectangles  # Return grasp details
    
    # Test function to run grasp detection on a sample image
    def testing_model(self):
        rgb_img = imread(r"/home/raval/robotic-grasping/cornell-dataset/01/pcd0138r.png")  
        depth_img = imread(r"/home/raval/robotic-grasping/cornell-dataset/01/pcd0138d.tiff")  
        original_rgb = rgb_img.copy()  
        quality, width, angle = self.run_grasp_detection(rgb_img, depth_img)  

        # Plot the metrics after detection
        self.plot_metrics(quality, width, angle)


# Entry point to test the model
if __name__ == "__main__":
    grasp_model = GraspDetectionNet()  # Initialize the grasp detection network
    grasp_model.testing_model()  # Test the model using sample images
