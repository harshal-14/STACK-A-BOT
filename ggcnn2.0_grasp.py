
import torch.nn.functional as F
from imageio import imread
import os
import sys

absPath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(absPath)

import cv2
import torch
import matplotlib.pyplot as plt
import argparse
import logging
import time
import numpy as np
import torch.utils.data



sys.path.append('/home/farhan/vbrm_project/src/grasp_model/grasp_model')
import post_process
from grasp import detect_grasps

class GraspDetectionGGCNN2():

    def __init__(self):
        model_path = absPath + "/trained_models/GGCNN/epoch_50_cornell_ggcnn2"
        self.network = torch.load(model_path, map_location='cpu')
        self.device = torch.device("cpu")  
        self.network.to(self.device)

    def load_and_preprocess_image(self, rgb_img, depth_img):
        # og_rgb = rgb_img.copy() 

        rgb_img_cropped, rgb_offset = self.crop_image(rgb_img)  
        depth_img_cropped, depth_offset = self.crop_image(depth_img)  

        # Depth image pre-processing
        depth_img_preprocessed = self.preprocess_depth_image(depth_img_cropped)

        return rgb_img_cropped, depth_img_preprocessed

    def crop_image(self, image, crop_size=(300, 300)):
        """ Crop image to a given size, keeping the center """
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        crop_width, crop_height = crop_size

        start_x = center_x - crop_width // 2
        start_y = center_y - crop_height // 2

        return image[start_y:start_y + crop_height, start_x:start_x + crop_width], (start_x, start_y)

    def preprocess_depth_image(self, depth_image):
        depth_crop = depth_image.copy()
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

        kernel = np.ones((3, 3), np.uint8)
        depth_nan_mask = cv2.dilate(depth_nan_mask, kernel, iterations=1)

        depth_crop[depth_nan_mask == 1] = 0

        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

        depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

        return depth_crop

    def format_inputs(self, rgb_img, depth_img):
        """ Normalize RGB and depth image and combine them as input tensor """
        rgb_img = rgb_img.astype(np.float32) / 255.0
        rgb_img -= rgb_img.mean()  
        rgb_img = rgb_img.transpose((2, 0, 1))  

        depth_img = np.clip(depth_img - np.mean(depth_img), -1, 1)
        depth_img = np.uint8(depth_img)

        combined_input = np.expand_dims(depth_img, 0)

        return torch.from_numpy(combined_input.astype(np.float32)).to(self.device)

    def predict_grasp(self, rgb_img, depth_img):
        """ Predict grasp from RGB and depth images """
        input_tensor = self.format_inputs(rgb_img, depth_img).unsqueeze(0)

        with torch.no_grad():
            output = self.network.forward(input_tensor)

            quality_img = output[0]   
            cos_angle = output[1]
            sin_angle = output[2]
            width_img = output[3]

        return post_process.post_process_output(quality_img, cos_angle, sin_angle, width_img)

    def visualize_grasp(self, rgb_img, grasp_rectangles):
        fig, ax = plt.subplots()
        ax.imshow(rgb_img)

        for grasp in grasp_rectangles:
            adjusted_center = (grasp.center[0], grasp.center[1])
            grasp.center = adjusted_center
            print(f"Adjusted Grasp Center: {adjusted_center}")
            grasp.plot(ax)

        ax.set_title('Predicted Grasp Rectangle')
        ax.axis('off')
        plt.savefig('grasp_prediction.png')
        plt.show()

    def run_grasp_detection(self, rgb, depth):
        rgb_img, depth_img = self.load_and_preprocess_image(rgb, depth)
        quality, angle, width = self.predict_grasp(rgb_img, depth_img)

        grasp_rectangles = detect_grasps(quality, angle, width, no_grasps=1)

        x_cen = round(grasp_rectangles[0].center[0], 5)
        y_cen = round(grasp_rectangles[0].center[1], 5)
        w = round(grasp_rectangles[0].width, 5)
        h = round(grasp_rectangles[0].length, 5)
        theta = round(grasp_rectangles[0].angle, 5)

        print(f"Grasp Center: ({x_cen}, {y_cen}), Width: {w}, Height: {h}, Angle: {theta}")
        self.visualize_grasp(rgb_img, grasp_rectangles)

        return [x_cen, y_cen, w, h, theta], depth_img, grasp_rectangles

    def testing_model(self):
        rgb_img = imread(r"/home/farhan/Desktop/VBRM_project/grasp_model/test_dataset_cornel/pcd0100r.png")
        depth_img = imread(r"/home/farhan/Desktop/VBRM_project/grasp_model/test_dataset_cornel/pcd0100d.tiff")

        original_rgb = rgb_img.copy()
        print(original_rgb.shape)

        _, _, grasp = self.run_grasp_detection(rgb_img, depth_img)


if __name__ == "__main__":
    grasp_model = GraspDetectionGGCNN()
    grasp_model.testing_model()
