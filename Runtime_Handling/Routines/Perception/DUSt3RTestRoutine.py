"""
This file is a test routine to integrate DUSt3R with the camera system.
"""

from ..Routine import Routine
from .. .Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Camera import Camera
from ....Components.Manipulator import Manipulator
from ....World.Geometry import Pose, Point
from ....Runtime_Handling.Routines.Manipulation.LinearInterpolationTS import LinearInterpolationTS
from scipy.spatial.transform import Rotation as R

import os
import sys
import numpy as np
import open3d as o3d
import cv2
from datetime import datetime
import time

# # Add the path to access dust3r_module
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
dust3r_module_path = os.path.join(project_root, "Algorithms", "dust3r")
sys.path.append(dust3r_module_path)

# Now import the module directly
import dust3r_module

class DUSt3RTestRoutine(Routine):
    """
    Routine to test DUSt3R integration with the camera system.
    """
    
    def __init__(self, num_angles=4, model_name="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt", output_dir="./output", image_dir=None):
        self.num_angles = num_angles
        self.model_name = model_name
        self.output_dir = output_dir
        self.image_dir = image_dir
        self.point_cloud = None
        self.camera = None
    
    def init(self, prev_outputs, parameters = None) -> Status:
        """Initialize and get camera reference."""
        try:
            # Get camera from singleton registry
            self.camera = get_singleton(Camera)
            
            # Create output directory
            os.makedirs(self.output_dir, exist_ok=True)
            
            return Status(Condition.Success)
        except Exception as e:
            return Status(Condition.Fault, 
                          err_msg=f"Error initializing DUSt3R test routine: {str(e)}", 
                          err_type=RuntimeError)
    
    # NOTE: 04/25/25 Harshal - Commenting out this method becuase obsolete. Keeping it for roll back if needed. 
    # def loop(self) -> Status:
    #     """Run DUSt3R to generate point cloud."""
    #     try:
    #         # Get image paths
    #         if self.image_dir:
    #             # Use existing images
    #             from pathlib import Path
    #             image_paths = []
    #             for ext in ['.jpg', '.jpeg', '.png']:
    #                 image_paths.extend([str(p) for p in Path(self.image_dir).glob(f'*{ext}')])
    #             image_paths.sort()
                
    #             if not image_paths:
    #                 return Status(Condition.Fault, 
    #                             err_msg=f"No images found in directory: {self.image_dir}", 
    #                             err_type=RuntimeError)
                
    #             print(f"Found {len(image_paths)} images in {self.image_dir}")
    #         else:
    #             # Use camera to capture images
    #             print("Using camera to capture images...")
    #             image_paths = self.camera.capture_images_interactive(
    #                 num_angles=self.num_angles,
    #                 output_dir=self.output_dir
    #             )
                
    #             if not image_paths:
    #                 return Status(Condition.Fault, 
    #                             err_msg="No images captured", 
    #                             err_type=RuntimeError)
                
    #             # DEBUGGING: Print captured image paths and verify files exist
    #             print("Captured images:")
    #             for i, path in enumerate(image_paths):
    #                 exists = os.path.exists(path)
    #                 print(f"  {i+1}: {path} - {'EXISTS' if exists else 'MISSING'}")
                    
    #                 # If file exists, check dimensions to verify it's not a composite
    #                 if exists:
    #                     img = cv2.imread(path)
    #                     print(f"    Image dimensions: {img.shape}")
            
    #         # Process images with DUSt3R - ensure we're using the specific method
    #         # that processes images, not the one that also captures them
    #         print("Processing images with DUSt3R...")
    #         self.point_cloud = dust3r_module.process_images_with_dust3r(
    #             image_paths, 
    #             model_name=self.model_name,
    #             output_dir=self.output_dir
    #         )
            
    #         # Check result
    #         if self.point_cloud is None or len(self.point_cloud) == 0:
    #             return Status(Condition.Fault, 
    #                         err_msg="Failed to generate point cloud", 
    #                         err_type=RuntimeError)
            
    #         print(f"Generated point cloud with {len(self.point_cloud)} points")
            
    #         # Visualize the point cloud
    #         dust3r_module.visualize_point_cloud(self.point_cloud)
            
    #         return Status(Condition.Success)
            
    #     except Exception as e:
    #         import traceback
    #         traceback.print_exc()
    #         return Status(Condition.Fault, 
    #                     err_msg=f"Error in DUSt3R processing: {str(e)}", 
    #                     err_type=RuntimeError)

    def loop(self) -> Status:
        """Run DUSt3R to generate point cloud using images captured at different robot positions."""
        try:
            if self.image_dir:
                from pathlib import Path
                image_paths = []
                for ext in ['.jpg', '.jpeg', '.png']:
                    image_paths.extend([str(p) for p in Path(self.image_dir).glob(f'*{ext}')])
                image_paths.sort()
                
                if not image_paths:
                    return Status(Condition.Fault, 
                                err_msg=f"No images found in directory: {self.image_dir}", 
                                err_type=RuntimeError)
                
                print(f"Found {len(image_paths)} images in {self.image_dir}")
            else:
                # Automatic capture at predetermined robot positions
                print("Beginning automated multi-view capture sequence...")
                image_paths = []
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                session_dir = os.path.join(self.output_dir, f"session_{timestamp}")
                os.makedirs(session_dir, exist_ok=True)
                
                # Get references to components
                manipulator = get_singleton(Manipulator)
                
                # Define view positions (robot end-effector poses)
                # These would be calibrated poses that give good coverage of the object
                # Create predefined poses for different views around the object
                # Assuming object is at center position [0, 0, 0.2]
                object_center = [0, 0, 0.2]
                radius = 0.4  # Distance from object
                height = 0.3  # Camera height
                
                view_poses = []
                for i in range(self.num_angles):
                    angle = (i * 2 * np.pi) / self.num_angles
                    x = object_center[0] + radius * np.cos(angle)
                    y = object_center[1] + radius * np.sin(angle)
                    z = object_center[2] + height
                    
                    # Point camera toward object center
                    rotation = R.from_euler('xyz', [0, np.pi/4, angle]).as_matrix()
                    view_poses.append(Pose(rotation, [x, y, z]))
                
                # Move to each position and capture
                for i, pose in enumerate(view_poses):
                    try:
                        print(f"Moving to view position {i+1}/{self.num_angles}")
                        
                        # Create a LinearInterpolationTS Routine to move to the position
                        move_routine = LinearInterpolationTS(pose, travel_time=2.0)
                        
                        # Initialize and run the routine
                        move_routine.init(None)
                        status = move_routine.loop()
                        
                        # Keep looping until movement is complete
                        while status.cond == Condition.In_Progress:
                            status = move_routine.loop()
                            time.sleep(0.1)  # Small delay to prevent CPU hogging
                        
                        # Finish the movement
                        move_routine.end()
                        
                        # Allow time for vibrations to settle
                        time.sleep(0.5)
                        
                        # Capture image
                        filename = f"view{i+1:02d}"
                        image_path = self.camera.capture_image(filename=filename, output_dir=session_dir)
                        image_paths.append(image_path)
                        print(f"Captured image at view position {i+1}/{self.num_angles}")
                        
                    except Exception as e:
                        print(f"Error at view position {i+1}: {str(e)}")
                        import traceback
                        traceback.print_exc()
                
                # Return to a safe position after capturing
                home_pose = manipulator.FK_Solver(np.array([[0], [0], [-np.pi/2], [0], [-np.pi/4], [0]]))
                home_routine = LinearInterpolationTS(home_pose, travel_time=2.0)
                home_routine.init(None)
                
                while home_routine.loop().cond == Condition.In_Progress:
                    time.sleep(0.1)
                
                home_routine.end()
                
                if not image_paths:
                    return Status(Condition.Fault, 
                                err_msg="No images captured during multi-view sequence", 
                                err_type=RuntimeError)
            
            # Process captured images with DUSt3R
            print(f"Processing {len(image_paths)} images with DUSt3R...")
            self.point_cloud = dust3r_module.process_images_with_dust3r(
                image_paths, 
                model_name=self.model_name,
                output_dir=self.output_dir
            )
            
            # Check result
            if self.point_cloud is None or len(self.point_cloud) == 0:
                return Status(Condition.Fault, 
                            err_msg="Failed to generate point cloud", 
                            err_type=RuntimeError)
            
            print(f"Successfully generated point cloud with {len(self.point_cloud)} points")
            
            # Visualize the point cloud
            dust3r_module.visualize_point_cloud(self.point_cloud)
            
            return Status(Condition.Success)
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            return Status(Condition.Fault, 
                        err_msg=f"Error in DUSt3R processing: {str(e)}", 
                        err_type=RuntimeError)
    
    def end(self) -> tuple[Status, dict]:
        """Return the generated point cloud."""
        outputs = {
            "point_cloud": self.point_cloud
        }
        return Status(Condition.Success), outputs
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        """Handle any faults during DUSt3R processing."""
        return Status(prev_status.cond, prev_status.err_msg, prev_status.err_type), {"point_cloud": None}