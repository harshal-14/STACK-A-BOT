import sys
import os
# Add the current directory to the system path
absPath = os.path.dirname(os.path.realpath(_file_))
sys.path.append(absPath)

import torch
import numpy as np
import cv2
import tifffile as tf
from imageio import imread
import scipy.ndimage as ndimage
import time
import math

sys.path.append(absPath)
from models.ggcnn import GGCNN  # Import the GGCNN model

class GraspDetectionGGCNN():

    def _init_(self):

        # Load pre-trained GGCNN model and prepare it for inference on the CPU
        model_path = absPath + "/trained_models/GGCNN/ggcnn_epoch_23_cornell"
        self.network = torch.load(model_path, map_location='cpu')
        self.device = torch.device("cpu")
        self.network.to(self.device)

    # Pre-process the depth image by cropping, normalizing, and filling missing data
    def pre_process(self, depth, crop_size, out_size=300, return_mask=False, crop_y_offset=0):
        imh, imw = depth.shape  # Get image dimensions

        # Crop the center part of the image based on the crop size
        depth_crop = depth[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                           (imw - crop_size) // 2:(imw - crop_size) // 2 + crop_size]

        # Add padding and handle missing values (NaN) in the depth image
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

        # Mask and inpaint areas with NaN or infinity in the depth data
        infinity_mask = np.isinf(depth_crop).astype(np.uint8)
        depth_nan_mask += infinity_mask
        kernel = np.ones((3, 3), np.uint8)
        depth_nan_mask = cv2.dilate(depth_nan_mask, kernel, iterations=1)
        depth_crop[depth_nan_mask == 1] = 0

        # Normalize and resize the depth image to the model's input size
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale
        depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop *= depth_scale
        depth_crop = cv2.resize(depth_crop, (out_size, out_size), cv2.INTER_AREA)

        # Return pre-processed image with optional mask
        if return_mask:
            depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
            depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), cv2.INTER_NEAREST)
            return depth_crop, depth_nan_mask
        else:
            return depth_crop

    # Predict the grasp pose from a depth image using the GGCNN model

    def predict(self, depth, crop_size, out_size, crop_y_offset):

        # Pre-process the input depth image for GGCNN
        depth = self.pre_process(depth, crop_size, out_size=300, return_mask=False, crop_y_offset=0)
        depth -= depth.mean()  # Normalize depth image
        depth = cv2.normalize(depth, None, -1, 1, cv2.NORM_MINMAX)

        # Convert depth image to tensor for the model
        tensor = torch.from_numpy(depth).float()
        tensor = torch.reshape(tensor, (1, 300, 300))

        # Run the GGCNN model and obtain the prediction outputs
        pred_out = self.network(tensor)
        pred_out = np.array([pred_out[0].detach().numpy(),
                             pred_out[1].detach().numpy(),
                             pred_out[2].detach().numpy(),
                             pred_out[3].detach().numpy()])
        return pred_out

    # Get the grasp coordinates, angle, and width from the prediction
    
    def get_grasp(self, rgb, depth, crop_size, out_size, crop_y_offset):

        # Convert RGB image to BGR and save depth and RGB images
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        np.savetxt('depth2.txt', depth, delimiter=',')
        cv2.imwrite("image2.png", rgb)

        # Predict grasp parameters from the depth image
        pred_out = self.predict(depth, crop_size, out_size, crop_y_offset)
        imh, imw = depth.shape
        argmax = np.argmax(pred_out[0])
        x = argmax % 300
        y = int(np.ceil(argmax / 300))

        # Extract grasp parameters: x, y, angle, and width
        width = pred_out[3][0][y][x] * 150
        angle = 0.5 * np.arctan2(pred_out[2][0][y][x], pred_out[1][0][x][y])
        grasp = np.array([x, y, angle, width])

        # Draw the predicted grasp on the RGB image
        self.draw(rgb, grasp, imw, imh, crop_y_offset)

        return grasp

    # Draw the predicted grasp on the image for visualization
    
    def draw(self, rgb, grasp, imw, imh, crop_y_offset):
        x = grasp[0]
        y = grasp[1]
        angle = grasp[2]
        width = grasp[3]

        # Adjust coordinates based on image dimensions and offsets
        x = x + imw * 0.5 - 150
        y = y + imh * 0.5 - 150 - crop_y_offset
        p1_x = int(x + (width * 0.5 * np.cos(angle)))
        p2_x = int(x - (width * 0.5 * np.cos(angle)))
        p1_y = int(y + (width * 0.5 * np.sin(angle)))
        p2_y = int(y - (width * 0.5 * np.sin(angle)))

        # Draw the grasp line and display the image
        grasp_gg = cv2.line(rgb, (p1_x, p1_y), (p2_x, p2_y), (255, 0, 0), 5)
        cv2.imshow("Grasp_GGCNN", grasp_gg)
        cv2.imwrite("ros_ggcnn_grasp.png", grasp_gg)
        cv2.waitKey(0)

    # A test function to run the model with sample input
    def testing_model(self):
        # depth = np.loadtxt('image', delimiter=',')
        rgb_img = imread(r"/home/raval/robotic-grasping/image_reshaped/book.png")
        depth_img = imread(r"/home/raval/robotic-grasping/image_reshaped/book.tiff")

        # Set crop and output size for the model
        imh, imw = depth_img.shape[:2]
        out_size = 300
        crop_y_offset = -40
        crop_size = 300

        # Load sample RGB image and get grasp prediction
        # rgb = cv2.imread("image1.png", cv2.IMREAD_COLOR)
        grasp = self.get_grasp(rgb_img, depth_img, crop_size, out_size, crop_y_offset)


if _name_ == "_main_":
    print(absPath)
    # Create an instance of the model and run the test function
    grasp_model = GraspDetectionGGCNN()
    grasp_model.testing_model()