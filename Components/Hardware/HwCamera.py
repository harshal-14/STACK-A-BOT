from ..Camera import Camera 
import numpy as np
import cv2
import depthai as dai
import time
import open3d as o3d
from datetime import datetime
import os
from pathlib import Path

class HwCamera(Camera):
    """Low-level integration & communication with OAK-D camera. 

    All Images/data is retrieved from the physical camera and made available through forward-facing API calls.\n
    HwCamera is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Camera)`. 
        
    Attributes:
        device(dai.Device): DepthAI device connection
        pipeline(dai.Pipeline): DepthAI pipeline for managing streams
        rgb_queue(dai.DataOutputQueue): Queue for RGB camera frames
        left_queue(dai.DataOutputQueue): Queue for left mono camera frames
        right_queue(dai.DataOutputQueue): Queue for right mono camera frames
        depth_queue(dai.DataOutputQueue): Queue for depth frames
    """

    def __init__(self):
        """Initialize the HwCamera class without connecting to a device."""
        print("------------Initializing HwCamera------------")
        self.device = None
        self.pipeline = None
        self.rgb_queue = None
        self.left_queue = None
        self.right_queue = None
        self.depth_queue = None
        # print("------------ HwCamera initialized------------")

    def create_pipeline(self):
        """Create and configure the DepthAI pipeline for the OAK-D camera for rgb abd depth streams."""
        pipeline = dai.Pipeline()

        # sources and o/ps
        rgb_cam = pipeline.create(dai.node.ColorCamera)
        left_cam = pipeline.create(dai.node.MonoCamera)
        right_cam = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        rgb_out = pipeline.create(dai.node.XLinkOut)
        left_out = pipeline.create(dai.node.XLinkOut)
        right_out = pipeline.create(dai.node.XLinkOut)
        depth_out = pipeline.create(dai.node.XLinkOut)

        rgb_out.setStreamName("rgb")
        left_out.setStreamName("left")
        right_out.setStreamName("right")
        depth_out.setStreamName("depth")

        # rgb cam properties
        rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A) # RGB camera
        rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P) # use max resolution
        rgb_cam.setPreviewSize(640, 480) # preview window size
        rgb_cam.setInterleaved(False)
        rgb_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

        # mono cam properties
        left_cam.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        left_cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right_cam.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        right_cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # stereo depth properties
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(640, 400)

        # linking
        rgb_cam.preview.link(rgb_out.input)
        left_cam.out.link(stereo.left)
        right_cam.out.link(stereo.right)

        left_cam.out.link(left_out.input)
        right_cam.out.link(right_out.input)
        stereo.depth.link(depth_out.input)

        return pipeline

    def bringup(self, **kwargs) -> int:
        """Initialize & cnonnect the OAK-D camera.
        
        Args:
            **kwargs: Additional arguments for connection configuration like port, ip, etc.
        Returns:
            int: Status code indicating success (0) or failure (non-zero)

        """
        return self.connect(kwargs=kwargs)

    def connect(self, **kwargs) -> int:
        """Connect to the OAK-D camera and initilize the DepthAI pipeline.
        
        Args:
            **kwargs: Additional arguments for connection configuration.

        Returns:
            int: Status code indicating success (0) or failure (non-zero)
        """
        try:
            self.pipeline = self.create_pipeline() 
            self.device = dai.Device(self.pipeline)

            self.rgb_queue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.left_queue = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.right_queue = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
            self.depth_queue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            print("------------HwCamera connected------------")
            return 0
        except Exception as e:
            print(f"Error connecting to HwCamera: {e}")
            return 1

    def disconnect(self, **kwargs) -> int:
        """Disconnect from the OAK-D camera.
        Args:
            **kwargs: Additional arguments for disconnection configuration.
        Returns:
            int: Status code indicating success (0) or failure (non-zero)
        """
        try:
            if self.device:
                self.device.close()
                self.device = None
                self.rgb_queue = None
                self.left_queue = None
                self.right_queue = None
                self.depth_queue = None
                print("------------HwCamera disconnected------------")
            return 0
        except Exception as e:
            print(f"Error disconnecting from HwCamera: {e}")
            return 1

    def get_RGB_image(self) -> np.ndarray:
        """Retrieve the RGB image from the camera.
        Returns:
            image(np.ndarray): a 3xHxW float32 ndarray with RGB channels.
        """
        if not self.device or not self.rgb_queue:
            raise RuntimeError("Camera not connected or RGB queue not initialized.")
        
        rgb_data = self.rgb_queue.get()
        rgb_frame = np.array(rgb_data.getData()).reshape((rgb_data.getHeight(), rgb_data.getWidth(), 3))
        
        rgb_frame = rgb_frame.astype(np.float32) / 255.0 # Normalize to [0, 1]
        rgb_frame = np.transpose(rgb_frame, (2, 0, 1)) # Change to CHW format
        
        return rgb_frame

    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        """Retrieve the mono images from the left and right cameras.
        
        Returns:
            images(tuple[np.ndarray, np.ndarray]): a pair of (1xHxW, 1xHxW) float32 ndarrays with mono images.
        """
        if not self.device or not self.left_queue or not self.right_queue:
            raise RuntimeError("Camera not connected or mono queues not initialized.")
        
        left_data = self.left_queue.get()
        right_data = self.right_queue.get()
        
        left_frame = np.array(left_data.getData()).reshape((left_data.getHeight(), left_data.getWidth()))
        right_frame = np.array(right_data.getData()).reshape((right_data.getHeight(), right_data.getWidth()))
        
        left_frame = left_frame.astype(np.float32) / 255.0 # Normalize to [0, 1]
        right_frame = right_frame.astype(np.float32) / 255.0 # Normalize to [0, 1]
    
        left_frame = np.expand_dims(left_frame, axis=0) # Add channel dimension (HW -> CHW)
        right_frame = np.expand_dims(right_frame, axis=0)
        
        return left_frame, right_frame
    
    def get_depth_data(self) -> np.ndarray:
        """Retrieve the depth data from the camera.
        
        Returns:
            depth(np.ndarray): a 1xHxW float32 ndarray with depth information.
        """
        depth_data = self.depth_queue.get()
        
        # Get raw data as numpy array
        raw_data = np.array(depth_data.getData(), dtype=np.uint8)
        
        # For 16-bit depth data (2 bytes per pixel)
        # Use frombuffer to correctly interpret 16-bit values
        depth_frame = np.frombuffer(raw_data, dtype=np.uint16)
        
        # Try to reshape to the expected dimensions
        try:
            depth_frame = depth_frame.reshape((depth_data.getHeight(), depth_data.getWidth()))
        except ValueError:
            # If that fails, calculate dimensions based on array size
            total_pixels = len(depth_frame)
            width = depth_data.getWidth()
            height = total_pixels // width
            depth_frame = depth_frame.reshape((height, width))
        
        # Convert to meters
        depth_map = depth_frame.astype(np.float32) / 1000.0
        
        # Add channel dimension (CHW format)
        depth_map = np.expand_dims(depth_map, axis=0)
        
        return depth_map
            
    def capture_images_interactive(self, num_angles=4, output_dir='.'):
        """Capture images from OAK-D Lite using the existing camera connection."""
        if not self.device or not self.rgb_queue:
            raise RuntimeError("Camera not connected")
        
        # Create output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(output_dir, f"session_{timestamp}")
        os.makedirs(session_dir, exist_ok=True)
        print(f"Images will be saved to: {session_dir}")
        
        image_paths = []
        
        try:
            print(f"\nWe'll capture {num_angles} different views of your scene.")
            print("For best results, move around the object between captures to get different angles.")
            
            # Clean up any existing windows
            cv2.destroyAllWindows()
            
            # Create a single window
            window_name = "Camera Preview"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 640, 480)
            
            for angle in range(num_angles):
                print(f"\n=== Preparing to capture angle {angle+1}/{num_angles} ===")
                print("Live preview started. Position the camera for a good view.")
                print("Press 'c' when ready to capture, or 'q' to quit.")
                
                while True:
                    # Get the latest frame using the existing queue
                    rgb_data = self.rgb_queue.get()
                    
                    # Use the built-in method to get a proper CV frame
                    rgb_frame = rgb_data.getCvFrame()
                    
                    # Add instruction text
                    preview = rgb_frame.copy()
                    cv2.putText(preview, f"Angle {angle+1}/{num_angles} - Press 'c' to capture", 
                                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Show preview
                    cv2.imshow(window_name, preview)
                    key = cv2.waitKey(1) & 0xFF
                    
                    if key == ord('c'):
                        # Get a fresh frame
                        rgb_data = self.rgb_queue.get()
                        rgb_frame = rgb_data.getCvFrame()
                        
                        # Save image
                        image_path = os.path.join(session_dir, f"view{angle+1:02d}.jpg")
                        cv2.imwrite(image_path, rgb_frame)
                        image_paths.append(image_path)
                        
                        # Verify image dimensions
                        saved_img = cv2.imread(image_path)
                        if saved_img is not None:
                            print(f"Saved image to {image_path}")
                            print(f"Image dimensions: {saved_img.shape}")
                        else:
                            print(f"Warning: Failed to verify saved image at {image_path}")
                        
                        # Show confirmation
                        confirm = rgb_frame.copy()
                        cv2.putText(confirm, "CAPTURED", (50, 50), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                        cv2.imshow(window_name, confirm)
                        cv2.waitKey(1000)
                        break
                        
                    elif key == ord('q'):
                        print("Capture process cancelled by user.")
                        cv2.destroyAllWindows()
                        return []
            
            cv2.destroyAllWindows()
            print(f"\nSuccessfully captured {len(image_paths)} angles!")
        
        except Exception as e:
            print(f"Error during image capture: {str(e)}")
            import traceback
            traceback.print_exc()
            cv2.destroyAllWindows()
            return []
        
        return image_paths
    
    def capture_images(self, filename = None, output_dir = '.'):
        """"
        Capture images from the OAK-D camera and save them to a specified directory.
        Args: 
            filename (str): Name of the file to save the captured images.
            output_dir (str): Directory to save the captured images.

        Returns:
            str: Path to the saved image file.
        """
        if not self.device or not self.rgb_queue:
            raise RuntimeError("Camera not connected")
            
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"image_{timestamp}"

        image_path = os.path.join(output_dir, f"{filename}.jpg")
        print(f"Image will be saved to: {image_path}") ## Check if this is correct!

        #get a fresh frame
        rgb_data = self.rgb_queue.get()
        rgb_frame = rgb_data.getCvFrame()
        cv2.imwrite(image_path, rgb_frame)

        return image_path