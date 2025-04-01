"""
Simplified wrapper for DUSt3R functionality that works with the existing architecture.
"""
import os
import subprocess
import numpy as np
import open3d as o3d
from datetime import datetime

class DUSt3RWrapper:
    """
    Wrapper for DUSt3R functionality that uses the existing scripts.
    """
    
    def __init__(self, model_name="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt", image_size=512):
        self.model_name = model_name
        self.image_size = image_size
        self.output_dir = os.path.join(os.getcwd(), "output", "dust3r")
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Path to the dust3r_module.py script
        self.dust3r_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dust3r_module.py")
    
    def process_images(self, image_paths):
        """
        Process captured images with DUSt3R to generate a point cloud.
        Uses the existing dust3r_module.py script via subprocess.
        
        Args:
            image_paths (list): List of paths to images
        
        Returns:
            numpy.ndarray: Point cloud as numpy array
        """
        if len(image_paths) < 2:
            print("Need at least 2 images for DUSt3R reconstruction")
            return None
        
        print(f"Processing {len(image_paths)} images with DUSt3R")
        
        # Create timestamp for output
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pcd_path = os.path.join(self.output_dir, f"dust3r_point_cloud_{timestamp}.ply")
        
        # Prepare arguments for dust3r_module.py
        args = [
            "python",
            self.dust3r_script,
            "--output-dir", self.output_dir,
            "--model", self.model_name,
            "--image-size", str(self.image_size)
        ]
        
        # Add image paths
        for img_path in image_paths:
            args.append(img_path)
        
        try:
            # Run dust3r_module.py as a subprocess
            print("Running DUSt3R module...")
            result = subprocess.run(args, check=True, capture_output=True, text=True)
            
            # Check if point cloud was created
            if os.path.exists(pcd_path):
                # Load point cloud using Open3D
                print(f"Loading point cloud from {pcd_path}")
                pcd = o3d.io.read_point_cloud(pcd_path)
                
                # Convert to numpy array
                points = np.asarray(pcd.points)
                print(f"Loaded {len(points)} points")
                
                return points
            else:
                print("Point cloud file not found at expected location")
                print("DUSt3R output:")
                print(result.stdout)
                print(result.stderr)
                return None
                
        except subprocess.CalledProcessError as e:
            print(f"Error running DUSt3R: {e}")
            print(e.stdout)
            print(e.stderr)
            return None
        except Exception as e:
            print(f"Error in DUSt3R processing: {str(e)}")
            import traceback
            traceback.print_exc()
            return None