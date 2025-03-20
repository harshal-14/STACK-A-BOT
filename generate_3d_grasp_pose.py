#!/usr/bin/env python3

import rclpy  # ROS2 client library for Python
from rclpy.node import Node  # Node base class to create ROS2 nodes
from sensor_msgs.msg import Image  # Import Image message type for receiving depth images
import cv2  # OpenCV library for image processing
import matplotlib.pyplot as plt  # For plotting if needed (currently unused)
import numpy as np  # For numerical operations

from std_msgs.msg import Float64MultiArray  # Message type for grasp bounding box data
from transforms3d import euler  # For converting Euler angles to quaternions
from cv_bridge import CvBridge  # Converts between ROS Image messages and OpenCV format

from geometry_msgs.msg import PoseStamped  # Message type for publishing 3D pose of grasp

# Define a ROS2 node class for generating 3D grasp
class Generate3DGrasp(Node):
    def __init__(self):
        super().__init__('grasp_generator_3d')  # Initialize the node with the name 'grasp_generator_3d'
        
        # Subscribe to the grasp bounding box topic
        self.grasp_box = self.create_subscription(
            Float64MultiArray,
            '/grasp_bounding_box',
            self.grasp_bounding_box_callback,
            10
        )
        # Subscribe to the depth image topic
        self.dp_img = self.create_subscription(
            Image,
            '/vbrm_project/depth_image',
            self.depth_image_callback,
            10
        )

        # Create a publisher to publish the generated 3D grasp pose
        self.grasp_3d = self.create_publisher(PoseStamped, '/grasp_real_3d', 10)
        self.box = None  # Placeholder for grasp bounding box data
        self.dp_img_val = None  # Placeholder for depth image data
        self.camera_frame = 'lens'  # Camera frame reference
        self.br = CvBridge()  # Initialize CvBridge for converting ROS images to OpenCV format

    # Callback for receiving grasp bounding box data
    def grasp_bounding_box_callback(self,msg):
        self.box = msg.data  # Store the received bounding box data
        self.get_logger().info(f"Grasp Box: {self.box}")  # Log the received box coordinates
        self.generate_real3d_grasp()  # Trigger 3D grasp generation

    # Callback for receiving depth image data
    def depth_image_callback(self,msg):
        self.dp_img_val = self.br.imgmsg_to_cv2(msg)  # Convert ROS Image to OpenCV format
        self.get_logger().info('Recieved -- now processing')  # Log that image processing is starting
        self.generate_real3d_grasp()  # Trigger 3D grasp generation

    # Function to generate the 3D grasp pose
    def generate_real3d_grasp(self):
        if self.box is None or self.dp_img_val is None:  # Check if both bounding box and depth image are available
            print("Grasp Box or depth image not found")  # Print error if data is missing
            return  # Exit the function if data is incomplete

        # Extract bounding box parameters
        x_center,y_center,width,height,theta = self.box

        # Get the height and width of the depth image
        img_h , img_w = self.dp_img_val.shape

        # Define camera intrinsic parameters (scaling factors)
        self.focal_x = 554  # Horizontal focal length
        self.focal_y = 554  # Vertical focal length
        self.cx = img_h//2   # Principal point x-coordinate
        self.cy = img_w//2   # Principal point y-coordinate

        # Convert 2D grasp coordinates to 3D real-world coordinates
        real_depth = self.dp_img_val[int(y_center), int(x_center)]  # Get depth value at grasp center
        pos_y = (x_center - self.cx) * real_depth / self.focal_x  # Calculate 3D y-coordinate
        pos_x = (y_center - self.cy) * real_depth / self.focal_y  # Calculate 3D x-coordinate
        pos_z = 1.0- real_depth  # Calculate object height (z-coordinate) above the environment

        # Create a PoseStamped message to hold the 3D grasp pose
        real_pose = PoseStamped()
        real_pose.header.frame_id = self.camera_frame  # Set the camera frame reference
        real_pose.header.stamp = self.get_clock().now().to_msg()  # Add timestamp

        # Set the 3D position of the grasp
        real_pose.pose.position.x = pos_x
        real_pose.pose.position.y = pos_y
        real_pose.pose.position.z = float(pos_z)

        # Calculate grasp orientation based on bounding box angle
        angle = np.radians(theta)  # Convert angle to radians
        quat = euler.euler2quat(np.pi, 0, ((angle % np.pi) - np.pi/2))  # Convert Euler angles to quaternion
        real_pose.pose.orientation.x = quat[1]
        real_pose.pose.orientation.y = quat[2]
        real_pose.pose.orientation.z = quat[3]
        real_pose.pose.orientation.w = quat[0]

        # Publish the 3D grasp pose
        self.grasp_3d.publish(real_pose)
        self.get_logger().info(f"generated grasp pose in 3D: {real_pose.pose}")  # Log the published pose

# Main function to initialize and run the ROS2 node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 Python client library
    grasp_pose_generator = Generate3DGrasp()  # Create an instance of the grasp generator node
    rclpy.spin(grasp_pose_generator)  # Keep the node running to handle incoming data
    grasp_pose_generator.destroy_node()  # Cleanup the node when done
    rclpy.shutdown()  # Shutdown ROS2

# Run the main function if the script is executed directly
if __name__ == '__main__':
    main()
