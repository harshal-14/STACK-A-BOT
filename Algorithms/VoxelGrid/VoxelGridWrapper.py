"""
Simplified wrapper for VoxelGrid functionality.
"""
import os
import subprocess
import numpy as np
import open3d as o3d
from datetime import datetime

class VoxelGridWrapper:
    """
    Wrapper for VoxelGrid functionality that uses the existing scripts.
    """
    
    def __init__(self, voxel_size=0.05):
        self.voxel_size = voxel_size
        self.output_dir = os.path.join(os.getcwd(), "output", "voxel_grid")
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Path to the improved_voxels.py script
        self.voxel_script = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                         "dust3r", "improved_voxels.py")
    
    def process_point_cloud(self, point_cloud, reset_grid=True):
        """
        Process a point cloud with the voxel grid.
        
        Args:
            point_cloud (np.ndarray): Point cloud data
            reset_grid (bool): Whether to reset the grid
            
        Returns:
            bool: Success status
        """
        # Save point cloud to a temporary file
        temp_pcd_path = os.path.join(self.output_dir, "temp_point_cloud.ply")
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud)
        
        # Save to file
        o3d.io.write_point_cloud(temp_pcd_path, pcd)
        
        # Prepare command to run improved_voxels.py
        output_mesh = os.path.join(self.output_dir, f"voxel_mesh_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply")
        
        # Command to run improved_voxels.py directly (assuming it has a main() function)
        cmd = [
            "python",
            self.voxel_script,
            "--point-cloud", temp_pcd_path,
            "--voxel-size", str(self.voxel_size),
            "--output", output_mesh,
            "--reset-grid", "true" if reset_grid else "false"
        ]
        
        try:
            print(f"Running voxel grid processing on point cloud with {len(point_cloud)} points...")
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            
            print("Voxel grid processing completed successfully")
            self.mesh_path = output_mesh
            return True
            
        except subprocess.CalledProcessError as e:
            print(f"Error running voxel grid: {e}")
            print(e.stdout)
            print(e.stderr)
            return False
        except Exception as e:
            print(f"Error in voxel grid processing: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def visualize_grid(self):
        """Visualize the current voxel grid."""
        # Use the existing mesh file if available
        if hasattr(self, 'mesh_path') and os.path.exists(self.mesh_path):
            try:
                mesh = o3d.io.read_triangle_mesh(self.mesh_path)
                print(f"Visualizing mesh from {self.mesh_path}...")
                o3d.visualization.draw_geometries([mesh])
                return True
            except Exception as e:
                print(f"Error visualizing mesh: {str(e)}")
                return False
        else:
            print("No mesh file available for visualization")
            return False