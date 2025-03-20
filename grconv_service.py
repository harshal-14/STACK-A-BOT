import sys
import os
absPath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(absPath)                                # Adding the path to the custom modules for grasp detection

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray       # Message type for publishing grasp data
from sensor_msgs.msg import Image                # Message type for RGB and depth image
from cv_bridge import CvBridge                   # Converts between ROS images and OpenCV images

from .grconv_grasp import *                # Relative import for GRConvNet grasp detection
from .ggcnn_grasp import *                 # Relative import for GGCNN grasp detection
from define_service.srv import GrConv      # Custom service definition for grasp model selection


class GraspService(Node):

    def __init__(self):
        
        super().__init__('Grasp_grconv_service')  # Initialize the node 

        # Subscription to the RGB image topic from the Realsense camera
        self.rgb_img_subs = self.create_subscription(
            Image,
            '/realsense/image_raw',
            self.rgb_callback,
            10)
        
        # Subscription to the depth image topic from the Realsense camera
        self.dp_img_sub = self.create_subscription(
            Image,
            '/realsense/depth/image_raw',
            self.depth_callback,
            10)

        # Publisher to send the grasp bounding box result
        self.grasp_box_pub = self.create_publisher(Float64MultiArray, '/grasp_bounding_box', 10)

        # Publisher for depth image in 3D world space
        self.depth_image_value = self.create_publisher(Image, '/vbrm_project/depth_image', 10)

        # Service definition that takes a model selection (GRConvNet or GGCNN) and returns the grasp data
        self.grasp_srv = self.create_service(GrConv, 'grconv_model', self.grasp_callback)

        # Initialize both GRConvNet and GGCNN models
        self.model1 = GraspDetectionNet()
        self.model2 = GraspDetectionGGCNN()
        self.rgb = None  # Placeholder for RGB image
        self.depth = None  # Placeholder for depth image

        self.br = CvBridge()  # Bridge for converting ROS image messages to OpenCV images

    # Service callback to process model requests
    def grasp_callback(self, request, response):
        # Use GRConvNet if specified by the user
        if request.model == "use_grconvnet":
            grasp, depth, _ = self.model1.run_grasp_detection(self.rgb, self.depth)

            # Prepare the grasp data to be sent as a ROS message
            grasp_msg = Float64MultiArray()
            grasp = np.array(grasp).astype(np.float64)
            grasp_msg.data = grasp.tolist()

            # Log the result and assign it to the response
            self.get_logger().info('Generated Grasp using GRConvNet')
            response.grasp = grasp_msg
            
            # Publish the grasp bounding box and depth image
            self.grasp_box_pub.publish(grasp_msg)
            self.depth_image_value.publish(self.br.cv2_to_imgmsg(depth))

        # Use GGCNN if specified by the user
        elif request.model == "use_ggcnn":
            grasp = self.model2.get_grasp(self.rgb, self.depth, 300, 300, 0)
            grasp_msg = Float64MultiArray()
            grasp = np.array(grasp).astype(np.float64)
            grasp_msg.data = grasp.tolist()

            # Log the result and assign it to the response
            self.get_logger().info('Generated Grasp using GGCNN 1.0')
            response.grasp = grasp_msg
            
            # Publish the grasp bounding box
            self.grasp_box_pub.publish(grasp_msg)

        else:
            print("No input from User")

        return response

    # Callback for receiving the RGB image
    def rgb_callback(self, msg):
        # Convert ROS image message to OpenCV format and adjust color space for model processing

        current_frame = self.br.imgmsg_to_cv2(msg)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        print('Got the RGB frame')
        self.rgb = current_frame  # Save the RGB image

    # Callback for receiving the depth image
    def depth_callback(self, msg):
        depth_frame = self.br.imgmsg_to_cv2(msg)
        self.depth = depth_frame  # Save the depth image


def main(args=None):
    print(absPath)
    rclpy.init(args=args)  # Initialize ROS communication
    gr_service = GraspService()  # Instantiate the service

    rclpy.spin(gr_service)  # Keep the node running to process incoming requests

    gr_service.destroy_node()  # Destroy the node when done
    rclpy.shutdown()  # Shutdown ROS communication
