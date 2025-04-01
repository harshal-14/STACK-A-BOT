# improved_voxels.py

import os
import numpy as np
import open3d as o3d
import time
import sys

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if project_root not in sys.path:
    sys.path.append(project_root)

from stack_a_bot.Algorithms.VoxelGrid.stabilized_voxel_grid import StabilizedVoxelGrid3D as BaseGrid

class StabilizedVoxelGrid3D(BaseGrid):
    def get_occupied_voxels(self, threshold=0.65):
        """Extract occupied voxels from the grid."""
        if self.occupancy_log_odds is None:
            return np.array([])
            
        # Convert log-odds to probabilities
        def log_odds_to_prob(l):
            return 1 - 1 / (1 + np.exp(l))
            
        occupancy_probs = log_odds_to_prob(self.occupancy_log_odds)
        
        # Find occupied indices
        occupied_indices = np.array(np.where(occupancy_probs > threshold)).T
        
        # Convert indices to 3D points
        occupied_points = np.array([self._index_to_point(idx) for idx in occupied_indices])
        
        return occupied_points

    def export_mesh(self, path):
        """Export the occupancy grid as a mesh."""
        occupied_points = self.get_occupied_voxels(threshold=0.65)
        if len(occupied_points) == 0:
            print("No occupied voxels to create mesh from.")
            return False
            
        # Create point cloud from occupied voxels
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(occupied_points)
        
        # Estimate normals
        pcd.estimate_normals()
        
        # Create mesh using Poisson reconstruction
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
        
        # Save mesh
        o3d.io.write_triangle_mesh(path, mesh)
        return True
    
    def process_frame(self, depth_frame, rgb_frame, confidence_frame=None):
        """Process a single frame and update the grid."""
        # Convert depth frame to point cloud
        point_cloud = self.depth_to_point_cloud(depth_frame, rgb_frame, confidence_frame)
        
        # Track camera motion
        if rgb_frame is not None and depth_frame is not None:
            transform = self._track_camera_motion(rgb_frame, depth_frame)
            self.camera_pose = self.camera_pose @ transform
        
        # Apply camera transformation to point cloud
        transformed_cloud = o3d.geometry.PointCloud()
        transformed_cloud.points = o3d.utility.Vector3dVector(np.asarray(point_cloud.points))
        if point_cloud.has_colors():
            transformed_cloud.colors = o3d.utility.Vector3dVector(np.asarray(point_cloud.colors))
        if point_cloud.has_normals():
            transformed_cloud.normals = o3d.utility.Vector3dVector(np.asarray(point_cloud.normals))
        
        # Stabilize the point cloud
        stabilized_cloud = self._fit_surfaces(transformed_cloud)
        
        # Update the occupancy grid
        self.update_grid(stabilized_cloud, sensor_origin=self.camera_pose[:3, 3])
        
        return stabilized_cloud
    
    def process_dust3r_point_cloud(self, dust3r_points, reset_grid=False):
        """
        Process a point cloud from DUSt3R and update the occupancy grid.
        
        Args:
            dust3r_points: Numpy array of points from DUSt3R
            reset_grid: Whether to reset the grid before updating
            
        Returns:
            Processed Open3D point cloud
        """
        if dust3r_points is None or len(dust3r_points) == 0:
            print("No points provided from DUSt3R")
            return None
            
        print(f"Processing {len(dust3r_points)} points from DUSt3R...")
        
        # Reset grid if requested
        if reset_grid:
            self.occupancy_log_odds = None
            self.camera_pose = np.eye(4)
        
        # Convert to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(dust3r_points)
        
        # First, perform advanced segmentation to isolate objects
        try:
            # from Algorithms.VoxelGrid.segment_point_cloud import segment_point_cloud
            sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            from VoxelGrid.segment_point_cloud import segment_point_cloud
            print("Applying advanced segmentation to point cloud...")
            segments = segment_point_cloud(pcd, voxel_size=self.voxel_size)
            
            # Optionally visualize the segmentation
            print(f"Segmentation found {len(segments)} components (including ground)")
            
            # Remove the ground plane (usually the largest segment)
            if len(segments) > 1:
                segment_sizes = [len(segment.points) for segment in segments]
                ground_idx = np.argmax(segment_sizes)
                
                # If the largest segment is more than 50% of all points, it's likely the ground
                if segment_sizes[ground_idx] > sum(segment_sizes) * 0.5:
                    print(f"Removing ground plane (segment {ground_idx} with {segment_sizes[ground_idx]} points)")
                    object_segments = segments.copy()
                    del object_segments[ground_idx]
                else:
                    object_segments = segments
                
                # Combine object segments into one point cloud
                object_cloud = o3d.geometry.PointCloud()
                for segment in object_segments:
                    object_cloud += segment
                
                # If we have objects, use them; otherwise use the full point cloud
                if len(object_cloud.points) > 100:
                    print(f"Using {len(object_cloud.points)} points from object segments")
                    pcd = object_cloud
                else:
                    print("Object segments too small, using full point cloud")
        except Exception as e:
            print(f"Advanced segmentation failed: {e}, using original point cloud")
            import traceback
            traceback.print_exc()
        
        # Now apply a conservative downsampling
        try:
            # Use a smaller voxel size for downsampling to preserve details
            voxel_ds_size = self.voxel_size * 0.2
            pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_ds_size)
            print(f"Downsampled to {len(pcd_downsampled.points)} points")
            
            # Only use downsampled point cloud if it has enough points
            if len(pcd_downsampled.points) > 1000:
                pcd = pcd_downsampled
            else:
                print("Downsampled point cloud too small, using original points")
        except Exception as e:
            print(f"Warning: Voxel downsampling failed: {e}")
        
        # Estimate normals which can help with surface reconstruction
        try:
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, max_nn=30))
            print("Normal estimation completed")
        except Exception as e:
            print(f"Normal estimation failed: {e}")
        
        # Initialize grid if needed
        if self.occupancy_log_odds is None:
            print("Initializing new occupancy grid...")
            
            # Get point bounds
            points = np.asarray(pcd.points)
            
            # Get tight bounds based on percentiles to exclude outliers
            p_min = np.percentile(points, 1, axis=0)
            p_max = np.percentile(points, 99, axis=0)
            
            # Add padding
            padding = self.voxel_size * 5  # 5 voxels of padding
            min_bound = p_min - padding
            max_bound = p_max + padding
            
            # Set grid bounds
            self.grid_bounds = (min_bound, max_bound)
            self.min_bound = min_bound
            
            # Calculate grid dimensions
            self.dims = np.ceil((max_bound - min_bound) / self.voxel_size).astype(int)
            print(f"Grid dimensions: {self.dims} ({np.prod(self.dims)} voxels)")
            
            # Create empty occupancy grid in log-odds form
            self.occupancy_log_odds = np.zeros(self.dims, dtype=np.float32)
        
        # Directly update the occupancy grid with the point cloud
        print("Updating occupancy grid with direct voxelization...")
        try:
            self._direct_voxelize_points(pcd)
            print("Grid updated successfully")
        except Exception as e:
            print(f"Error updating grid: {e}")
            import traceback
            traceback.print_exc()
        
        return pcd
        
    def _direct_voxelize_points(self, point_cloud):
        """
        Directly voxelize points into the occupancy grid without ray casting.
        This is more appropriate for pre-optimized point clouds like those from DUSt3R.
        
        Args:
            point_cloud: Open3D point cloud
        """
        if self.occupancy_log_odds is None:
            print("Error: Grid not initialized")
            return
            
        # Get points as numpy array
        points = np.asarray(point_cloud.points)
        
        # Convert points to grid indices
        indices = np.floor((points - self.min_bound) / self.voxel_size).astype(int)
        
        # Filter out points outside grid bounds
        valid_mask = np.all((indices >= 0) & (indices < self.dims), axis=1)
        valid_indices = indices[valid_mask]
        
        print(f"Voxelizing {len(valid_indices)} points into grid")
        
        # Create a temporary grid to track which voxels have been processed
        processed = np.zeros(self.dims, dtype=bool)
        
        # We'll use a small kernel to create a thin shell instead of a solid volume
        # Only mark the voxel itself and immediate neighbors that are empty
        for idx in valid_indices:
            # Mark the voxel containing the point
            idx_tuple = tuple(idx)
            self.occupancy_log_odds[idx_tuple] = max(self.occupancy_log_odds[idx_tuple], self.l_occ)
            processed[idx_tuple] = True
            
            # Only create a thin shell, don't fill the volume
            # Check only the 6-connected neighbors (not diagonals)
            neighbors = [
                (0, 0, 1), (0, 0, -1),  # above and below
                (0, 1, 0), (0, -1, 0),  # front and back
                (1, 0, 0), (-1, 0, 0)   # left and right
            ]
            
            for dx, dy, dz in neighbors:
                neighbor_idx = idx + np.array([dx, dy, dz])
                neighbor_tuple = tuple(neighbor_idx)
                
                # Only process if in bounds and not already processed
                if (np.all((neighbor_idx >= 0) & (neighbor_idx < self.dims)) and 
                    not processed[neighbor_tuple]):
                    
                    # Add a weaker evidence of occupancy
                    self.occupancy_log_odds[neighbor_tuple] += self.l_occ * 0.3
                    # Don't mark as processed so other points can still contribute
        
        # Clip values to maintain numerical stability
        self.occupancy_log_odds = np.clip(self.occupancy_log_odds, self.l_min, self.l_max)
        
        # Count occupied voxels
        def log_odds_to_prob(l):
            return 1 - 1 / (1 + np.exp(l))
        
        # Use a lower threshold for counting to show all potentially occupied voxels
        occupancy_probs = log_odds_to_prob(self.occupancy_log_odds)
        
        thresholds = [0.65, 0.5, 0.3]
        for threshold in thresholds:
            occupied_count = np.sum(occupancy_probs > threshold)
            print(f"Grid contains {occupied_count} occupied voxels at threshold {threshold}")
    
    def visualize_occupancy_grid(self, threshold=0.65):
        """
        Visualize the current occupancy grid.
        
        Args:
            threshold: Occupancy probability threshold (0-1)
        """
        occupied_points = self.get_occupied_voxels(threshold=threshold)
        
        if len(occupied_points) == 0:
            print("No occupied voxels to visualize.")
            return
            
        print(f"Visualizing occupancy grid with {len(occupied_points)} occupied voxels...")
            
        # Create point cloud from occupied voxels
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(occupied_points)
        
        # Generate colors based on height
        colors = np.zeros((len(occupied_points), 3))
        min_z = np.min(occupied_points[:, 2])
        max_z = np.max(occupied_points[:, 2])
        z_range = max(0.001, max_z - min_z)
        
        for i, point in enumerate(occupied_points):
            norm_z = (point[2] - min_z) / z_range
            if norm_z < 0.33:
                colors[i] = [0, norm_z * 3, 1]
            elif norm_z < 0.66:
                colors[i] = [(norm_z - 0.33) * 3, 1, 1 - (norm_z - 0.33) * 3]
            else:
                colors[i] = [1, 1 - (norm_z - 0.66) * 3, 0]
        
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Create voxel grid for visualization
        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
            pcd, voxel_size=self.voxel_size)
            
        # Create coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0])
            
        # Visualize
        print("Opening visualization window... (close the window to continue)")
        o3d.visualization.draw_geometries([voxel_grid, coord_frame])
        print("Visualization closed")
    
    def run_with_visualization(self):
        """Run with live visualization."""
        import depthai as dai
        import cv2
        import numpy as np
        from collections import deque
        
        # Initialize Open3D visualizer
        vis = o3d.visualization.Visualizer()
        vis.create_window("Stabilized 3D Occupancy Grid", 1024, 768)
        
        # Configure render options
        render_opt = vis.get_render_option()
        render_opt.background_color = np.array([0.1, 0.1, 0.1])
        render_opt.point_size = 3.0
        
        # Add coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.5, origin=[0, 0, 0])
        vis.add_geometry(coord_frame)
        
        # Add point cloud placeholder
        pcd = o3d.geometry.PointCloud()
        vis.add_geometry(pcd)
        
        # Voxel grid visualization
        voxel_grid_geometry = o3d.geometry.VoxelGrid()
        voxel_grid_added = False
        
        # Connect to device
        with dai.Device(self.pipeline) as device:
            # Get output queues
            rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            depth_queue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            conf_queue = device.getOutputQueue(name="confidence", maxSize=4, blocking=False)
            
            # For FPS calculation
            fps_history = deque(maxlen=10)
            
            while True:
                # Get frames
                rgb_frame = rgb_queue.get().getCvFrame()
                depth_frame = depth_queue.get().getFrame()
                
                try:
                    confidence_frame = conf_queue.get().getFrame()
                except:
                    confidence_frame = None
                
                # Process frame
                start_time = time.time()
                filtered_pcd = self.process_frame(depth_frame, rgb_frame, confidence_frame)
                process_time = time.time() - start_time
                
                # Calculate FPS
                fps_history.append(1.0 / max(0.001, process_time))
                fps = np.mean(fps_history)
                
                # Update visualization
                if len(filtered_pcd.points) > 10:
                    pcd.points = o3d.utility.Vector3dVector(np.asarray(filtered_pcd.points))
                    pcd.colors = o3d.utility.Vector3dVector(np.asarray(filtered_pcd.colors))
                    vis.update_geometry(pcd)
                    
                    # Extract and visualize occupied voxels
                    occupied_points = self.get_occupied_voxels(threshold=0.65)
                    
                    if len(occupied_points) > 0:
                        # Create voxel grid visualization
                        occupied_pcd = o3d.geometry.PointCloud()
                        occupied_pcd.points = o3d.utility.Vector3dVector(occupied_points)
                        
                        # Generate colors based on height
                        colors = np.zeros((len(occupied_points), 3))
                        min_z = np.min(occupied_points[:, 2])
                        max_z = np.max(occupied_points[:, 2])
                        z_range = max(0.001, max_z - min_z)
                        
                        for i, point in enumerate(occupied_points):
                            norm_z = (point[2] - min_z) / z_range
                            if norm_z < 0.33:
                                colors[i] = [0, norm_z * 3, 1]
                            elif norm_z < 0.66:
                                colors[i] = [(norm_z - 0.33) * 3, 1, 1 - (norm_z - 0.33) * 3]
                            else:
                                colors[i] = [1, 1 - (norm_z - 0.66) * 3, 0]
                        
                        occupied_pcd.colors = o3d.utility.Vector3dVector(colors)
                        
                        # Create voxel grid from point cloud
                        voxel_grid_geometry = o3d.geometry.VoxelGrid.create_from_point_cloud(
                            occupied_pcd, voxel_size=self.voxel_size)
                        
                        if not voxel_grid_added:
                            vis.add_geometry(voxel_grid_geometry)
                            voxel_grid_added = True
                        else:
                            vis.update_geometry(voxel_grid_geometry)
                
                # Update visualization
                vis.poll_events()
                vis.update_renderer()
                
                # Display video feed
                cv2.putText(rgb_frame, f"FPS: {fps:.1f}", (10, 30), 
                             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("RGB", rgb_frame)
                
                # Show depth
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_frame, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                cv2.imshow("Depth", depth_colormap)
                
                # Handle keyboard input
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
        
        # Clean up
        cv2.destroyAllWindows()
        vis.destroy_window()