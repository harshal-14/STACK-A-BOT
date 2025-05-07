from ..Routine import Routine
from . ..Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Camera import Camera
import os
import sys
import numpy as np
import open3d as o3d

# Add the path to access improved_voxels
# project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
# dust3r_path = os.path.join(project_root, "Algorithms", "dust3r")
# sys.path.append(dust3r_path)

# Now import the module directly
from improved_voxels import StabilizedVoxelGrid3D

class VoxelGridTestRoutine(Routine):
    """
    Routine to test VoxelGrid integration with the camera system.
    """
    
    def __init__(self, voxel_size=0.05, output_dir="./output", num_frames=5, use_dust3r_cloud=True):
        self.voxel_size = voxel_size
        self.output_dir = output_dir
        self.num_frames = num_frames
        self.use_dust3r_cloud = use_dust3r_cloud
        self.voxel_grid = None
        self.camera = None
        self.frame_count = 0
        self.is_complete = False

    def init(self, prev_outputs, parameters = None) -> Status:
        """Initialize voxel grid and camera reference."""
        try:
            # Get camera from singleton registry
            self.camera = get_singleton(Camera)
            
            # Create output directory
            os.makedirs(self.output_dir, exist_ok=True)
            
            # Initialize VoxelGrid
            self.voxel_grid = StabilizedVoxelGrid3D(voxel_size=self.voxel_size)
            
            # Check for point cloud from previous routine
            if prev_outputs and "point_cloud" in prev_outputs and self.use_dust3r_cloud:
                self.point_cloud = prev_outputs["point_cloud"]
                if self.point_cloud is not None and len(self.point_cloud) > 0:
                    print(f"Using point cloud from DUSt3R with {len(self.point_cloud)} points")
                    # Process point cloud
                    pcd = self.voxel_grid.process_dust3r_point_cloud(self.point_cloud, reset_grid=True)
                    if pcd is not None:
                        print("Successfully processed DUSt3R point cloud")
                        self.is_complete = True  # Skip camera depth processing
                    else:
                        print("Failed to process DUSt3R point cloud, will try camera depth instead")
            
            return Status(Condition.Success)
        except Exception as e:
            return Status(Condition.Fault, 
                          err_msg=f"Error initializing VoxelGrid test routine: {str(e)}", 
                          err_type=RuntimeError)
    
    def loop(self) -> Status:
        """Update voxel grid from camera or point cloud."""
        try:
            # If we already processed a point cloud, we're done
            if self.is_complete:
                return Status(Condition.Success)
            
            # If we've processed enough frames, we're done
            if self.frame_count >= self.num_frames:
                return Status(Condition.Success)
            
            # Get depth and RGB frames from camera
            print(f"Processing frame {self.frame_count+1}/{self.num_frames}...")
            try:
                depth_frame = self.camera.get_depth_data()[0]  # Get first channel
                rgb_frame = np.transpose(self.camera.get_RGB_image(), (1, 2, 0))  # CHW to HWC
                
                # Process depth frame
                self.voxel_grid.process_frame(depth_frame, rgb_frame)
            except Exception as e:
                print(f"Error getting camera frames: {str(e)}")
                import traceback
                traceback.print_exc()
            
            self.frame_count += 1
            
            # If still processing frames, return in progress
            if self.frame_count < self.num_frames:
                return Status(Condition.In_Progress)
            
            return Status(Condition.Success)
            
        except Exception as e:
            return Status(Condition.Fault, 
                         err_msg=f"Error processing VoxelGrid: {str(e)}", 
                         err_type=RuntimeError)
    
    def end(self) -> tuple[Status, dict]:
        """Visualize and return the voxel grid."""
        try:
            # Export mesh
            mesh_path = os.path.join(self.output_dir, f"voxel_mesh_{self.frame_count}.ply")
            self.voxel_grid.export_mesh(mesh_path)
            
            # Visualize grid
            print("Visualizing voxel grid...")
            self.voxel_grid.visualize_occupancy_grid()
            
            outputs = {
                "voxel_grid": self.voxel_grid,
                "mesh_path": mesh_path
            }
            
            return Status(Condition.Success), outputs
        except Exception as e:
            print(f"Error in VoxelGrid end routine: {str(e)}")
            import traceback
            traceback.print_exc()
            return Status(Condition.Fault, 
                         err_msg=f"Error visualizing VoxelGrid: {str(e)}", 
                         err_type=RuntimeError), {"voxel_grid": self.voxel_grid}
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        """Handle any faults during VoxelGrid processing."""
        return Status(prev_status.cond, prev_status.err_msg, prev_status.err_type), {"voxel_grid": self.voxel_grid}