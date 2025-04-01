"""
This file is a test routine to integrate DUSt3R with the camera system.
"""

from ..Routine import Routine
from .. .Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Camera import Camera
import os
import sys
import numpy as np
import open3d as o3d
import cv2

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
    
    # Bridge methods for RoutineScheduler, useful maintaining Routine schedular and 
    def _init(self, prev_outputs):
        return self.init(prev_outputs)
        
    def _loop(self):
        return self.loop()
        
    def _end(self):
        return self.end()
        
    def _handle_fault(self, prev_status=None):
        return self.handle_fault(prev_status)
    
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
            
    def loop(self) -> Status:
        """Run DUSt3R to generate point cloud."""
        try:
            # Get image paths
            if self.image_dir:
                # Use existing images
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
                # Use camera to capture images
                print("Using camera to capture images...")
                image_paths = self.camera.capture_images_interactive(
                    num_angles=self.num_angles,
                    output_dir=self.output_dir
                )
                
                if not image_paths:
                    return Status(Condition.Fault, 
                                err_msg="No images captured", 
                                err_type=RuntimeError)
                
                # DEBUGGING: Print captured image paths and verify files exist
                print("Captured images:")
                for i, path in enumerate(image_paths):
                    exists = os.path.exists(path)
                    print(f"  {i+1}: {path} - {'EXISTS' if exists else 'MISSING'}")
                    
                    # If file exists, check dimensions to verify it's not a composite
                    if exists:
                        img = cv2.imread(path)
                        print(f"    Image dimensions: {img.shape}")
            
            # Process images with DUSt3R - ensure we're using the specific method
            # that processes images, not the one that also captures them
            print("Processing images with DUSt3R...")
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
            
            print(f"Generated point cloud with {len(self.point_cloud)} points")
            
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