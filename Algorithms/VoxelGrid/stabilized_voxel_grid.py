import depthai as dai
import numpy as np
import cv2
import open3d as o3d
import time
from collections import deque
from scipy import ndimage
import math
import random

class StabilizedVoxelGrid3D:
    def __init__(self, voxel_size=0.05, grid_bounds=None):
        """
        Initialize a stabilized 3D occupancy grid with jitter reduction techniques.
        
        Args:
            voxel_size: Size of each voxel in meters (larger = more stable, less detail)
            grid_bounds: Optional bounds as ((min_x, min_y, min_z), (max_x, max_y, max_z))
                         If None, will be determined from first point cloud
        """
        self.voxel_size = voxel_size
        self.grid_bounds = grid_bounds
        
        # Probabilistic occupancy grid parameters (log-odds representation)
        self.l_occ = np.log(0.65/0.35)  # Log odds for occupied measurement
        self.l_free = np.log(0.35/0.65)  # Log odds for free measurement
        self.l_prior = 0.0  # Log odds prior (0 = 0.5 probability)
        self.l_min = -5.0  # Min log-odds threshold
        self.l_max = 5.0   # Max log-odds threshold
        
        # Grid state
        self.occupancy_log_odds = None
        self.dims = None
        self.min_bound = None
        
        # Camera intrinsics for OAK-D Lite (from calibration)
        self.fx = 557.477
        self.fy = 557.488
        self.cx = 317.666
        self.cy = 198.185
        
        # Temporal filtering
        self.smoothed_depth = None
        self.depth_history = deque(maxlen=3)  # Store last 3 depth frames
        self.camera_pose = np.eye(4)  # Identity transformation
        self.prev_rgb = None
        self.keyframe_rgb = None
        self.keyframe_count = 0
        
        # Noise reduction parameters
        self.depth_confidence_threshold = 120  # Higher threshold for better quality (0-255)
        self.depth_bilateral_d = 7        # Diameter of bilateral filter
        self.depth_bilateral_sigma_color = 80.0
        self.depth_bilateral_sigma_space = 7.0
        
        # Point cloud filtering
        self.voxel_downsample_size = voxel_size * 0.75
        self.outlier_radius = voxel_size * 3.0
        self.outlier_min_neighbors = 3
        
        # Motion tracking
        self.feature_count = 500
        self.feature_quality = 0.01
        self.min_feature_distance = 10
        self.prev_kp = None
        self.prev_des = None
        self.prev_points = None
        self.max_drift = 0.02  # Maximum allowed drift per frame in meters
        
        # Create pipeline
        self.pipeline = self._create_pipeline()
        
    def _create_pipeline(self):
        """Create a pipeline for depth and RGB with confidence."""
        pipeline = dai.Pipeline()
        
        # RGB Camera for colorization
        colorCam = pipeline.create(dai.node.ColorCamera)
        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        
        colorCam.setPreviewSize(640, 400)
        colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        colorCam.setInterleaved(False)
        colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        colorCam.preview.link(xoutRgb.input)
        
        # Mono cameras for stereo depth
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        
        # Create stereo depth
        stereo = pipeline.create(dai.node.StereoDepth)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        
        # Add confidence output
        xoutConfidence = pipeline.create(dai.node.XLinkOut)
        xoutConfidence.setStreamName("confidence")
        
        # Configure stereo depth
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)  # Better for static mapping
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Align with RGB
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setLeftRightCheck(True)  # Enable LR-check for better accuracy
        stereo.setSubpixel(True)  # Enable subpixel interpolation
        stereo.setExtendedDisparity(True)  # Enable extended disparity for close objects
        stereo.setRectifyEdgeFillColor(0)  # Black, on edges
        stereo.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)  # Apply median filter
        stereo.setConfidenceThreshold(200)  # Higher threshold for better quality
        
        # Link nodes
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        stereo.depth.link(xoutDepth.input)
        stereo.confidenceMap.link(xoutConfidence.input)
        
        return pipeline
    
    def _preprocess_depth(self, depth_frame, confidence_map=None):
        """
        Enhanced preprocessing of depth with temporal and spatial filtering.
        
        Args:
            depth_frame: Raw depth frame
            confidence_map: Optional confidence map (0-255)
            
        Returns:
            Filtered depth frame
        """
        # Confidence thresholding first
        if confidence_map is not None:
            mask = confidence_map < self.depth_confidence_threshold
            filtered_depth = depth_frame.copy()
            filtered_depth[mask] = 0
        else:
            filtered_depth = depth_frame.copy()
        
        # Apply spatial gradient consistency check
        sobelx = cv2.Sobel(filtered_depth, cv2.CV_32F, 1, 0, ksize=3)
        sobely = cv2.Sobel(filtered_depth, cv2.CV_32F, 0, 1, ksize=3)
        gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
        
        # Threshold on depth discontinuities (adjust based on scene)
        max_gradient = 80.0  # Higher allows more edges
        filtered_depth[gradient_magnitude > max_gradient] = 0
        
        # Apply bilateral filter to preserve edges while reducing noise
        # Convert to 8-bit for filtering
        depth_scaled = cv2.normalize(filtered_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_filtered = cv2.bilateralFilter(
            depth_scaled, 
            self.depth_bilateral_d,
            self.depth_bilateral_sigma_color,
            self.depth_bilateral_sigma_space
        )
        # Scale back to original range
        depth_filtered = cv2.normalize(depth_filtered, None, 0, np.max(filtered_depth), cv2.NORM_MINMAX, cv2.CV_16U)
        
        # Apply temporal filtering if history exists
        if self.smoothed_depth is None:
            self.smoothed_depth = depth_filtered.copy()
        else:
            # Apply exponential moving average
            alpha = 0.35  # Smoothing factor (lower = more smoothing but more latency)
            mask = (depth_filtered > 0) & (self.smoothed_depth > 0)
            
            # Update existing measurements with EMA
            self.smoothed_depth[mask] = (1-alpha) * self.smoothed_depth[mask] + alpha * depth_filtered[mask]
            
            # For newly valid pixels, just use the current depth
            new_valid = (depth_filtered > 0) & (self.smoothed_depth == 0)
            self.smoothed_depth[new_valid] = depth_filtered[new_valid]
        
        # Add to history for future filtering
        self.depth_history.append(depth_filtered.copy())
        
        # Fill small holes using morphological operations
        kernel = np.ones((3, 3), np.uint8)
        depth_mask = (self.smoothed_depth > 0).astype(np.uint8)
        filled_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_CLOSE, kernel)
        
        # Use inpainted depth only for small holes
        depth_inpainted = cv2.inpaint(
            self.smoothed_depth.astype(np.uint8), 
            (1 - filled_mask).astype(np.uint8), 
            3, 
            cv2.INPAINT_NS
        )
        
        # Combine original and inpainted depth
        result = self.smoothed_depth.copy()
        hole_mask = (filled_mask > 0) & (depth_mask == 0)
        result[hole_mask] = depth_inpainted[hole_mask]
        
        return result
    
    def _track_camera_motion(self, rgb_frame, depth_frame):
        """
        Track camera motion between frames for stabilization.
        
        Args:
            rgb_frame: Current RGB frame
            depth_frame: Current depth frame
            
        Returns:
            4x4 transformation matrix
        """
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)
        
        # Initialize feature detector
        orb = cv2.ORB_create(nfeatures=1000)
        
        # First frame case
        if self.prev_rgb is None:
            self.prev_rgb = rgb_frame.copy()
            self.prev_kp, self.prev_des = orb.detectAndCompute(gray, None)
            return np.eye(4)  # Identity transform
        
        # Detect keypoints and compute descriptors
        kp2, des2 = orb.detectAndCompute(gray, None)
        
        # If not enough features were found
        if self.prev_des is None or des2 is None or len(self.prev_des) < 10 or len(des2) < 10:
            self.prev_rgb = rgb_frame.copy()
            self.prev_kp, self.prev_des = kp2, des2
            return np.eye(4)
        
        # Match features
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = matcher.match(self.prev_des, des2)
        
        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)
        
        # Take top 50 matches
        good_matches = matches[:min(50, len(matches))]
        
        # Extract matched keypoints
        prev_pts = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
        curr_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches])
        
        if len(prev_pts) < 4 or len(curr_pts) < 4:
            self.prev_rgb = rgb_frame.copy()
            self.prev_kp, self.prev_des = kp2, des2
            return np.eye(4)
        
        # Estimate the essential matrix
        E, mask = cv2.findEssentialMat(prev_pts, curr_pts, 
                                      focal=self.fx, 
                                      pp=(self.cx, self.cy), 
                                      method=cv2.RANSAC, 
                                      prob=0.999, 
                                      threshold=1.0)
        
        if E is None:
            self.prev_rgb = rgb_frame.copy()
            self.prev_kp, self.prev_des = kp2, des2
            return np.eye(4)
        
        # Recover rotation and translation from essential matrix
        _, R, t, _ = cv2.recoverPose(E, prev_pts, curr_pts, 
                                     focal=self.fx, pp=(self.cx, self.cy), mask=mask)
        
        # Create 4x4 transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = t.reshape(3)
        
        # Apply scale correction using depth information
        # This is important to avoid drift
        scale = self._estimate_scale(prev_pts, curr_pts, depth_frame)
        transform[:3, 3] *= scale
        
        # Limit translation to prevent extreme jumps (outlier rejection)
        max_translation = self.max_drift
        if np.linalg.norm(transform[:3, 3]) > max_translation:
            scaling_factor = max_translation / np.linalg.norm(transform[:3, 3])
            transform[:3, 3] *= scaling_factor
        
        # Update reference frame
        self.prev_rgb = rgb_frame.copy()
        self.prev_kp, self.prev_des = kp2, des2
        
        return transform
    
    def _estimate_scale(self, prev_pts, curr_pts, depth_frame):
        """
        Estimate scale factor from depth to avoid drift.
        
        Args:
            prev_pts: Previous feature points
            curr_pts: Current feature points
            depth_frame: Current depth frame
            
        Returns:
            Estimated scale factor
        """
        # Default scale
        default_scale = 0.05
        
        # Sample a few points for depth-based scale estimation
        sample_count = min(10, len(prev_pts))
        if sample_count < 3:
            return default_scale
            
        # Randomly sample points
        indices = random.sample(range(len(prev_pts)), sample_count)
        
        depths = []
        for idx in indices:
            x, y = int(curr_pts[idx][0]), int(curr_pts[idx][1])
            if 0 <= x < depth_frame.shape[1] and 0 <= y < depth_frame.shape[0]:
                depth = depth_frame[y, x]
                if depth > 0:
                    depths.append(depth / 1000.0)  # Convert to meters
        
        if not depths:
            return default_scale
            
        # Use median depth as a robust estimate
        median_depth = np.median(depths)
        
        # Scale inversely proportional to depth (closer = more movement)
        scale = min(0.2, max(0.01, 0.05 / max(0.1, median_depth)))
        return scale
    
    def depth_to_point_cloud(self, depth_frame, rgb_frame=None, confidence_map=None):
        """
        Convert depth frame to filtered point cloud.
        
        Args:
            depth_frame: Depth image
            rgb_frame: Optional RGB image for colorization
            confidence_map: Optional confidence map (0-255)
        
        Returns:
            Open3D point cloud object
        """
        # Preprocess depth to reduce noise
        filtered_depth = self._preprocess_depth(depth_frame, confidence_map)
        
        rows, cols = filtered_depth.shape
        points = []
        colors = []
        
        # Process depth image with variable sampling (closer points sampled more densely)
        step = 3  # Base step size (bigger = faster but less detailed)
        for y in range(0, rows, step):
            for x in range(0, cols, step):
                depth_value = filtered_depth[y, x]
                
                # Skip invalid depth
                if depth_value <= 0:
                    continue
                
                # Convert to meters
                z = depth_value / 1000.0
                
                # Skip points that are too far or too close
                if z > 3.5 or z < 0.3:
                    continue
                
                # Convert to 3D coordinates
                x_3d = (x - self.cx) * z / self.fx
                y_3d = (y - self.cy) * z / self.fy
                z_3d = z
                
                points.append([x_3d, y_3d, z_3d])
                
                # Add color if RGB frame is available
                if rgb_frame is not None:
                    # Get color - need to resize RGB frame to match depth
                    rgb_y = int(y * rgb_frame.shape[0] / rows)
                    rgb_x = int(x * rgb_frame.shape[1] / cols)
                    
                    if 0 <= rgb_y < rgb_frame.shape[0] and 0 <= rgb_x < rgb_frame.shape[1]:
                        color = rgb_frame[rgb_y, rgb_x] / 255.0  # Normalize to 0-1
                        colors.append(color)
                    else:
                        colors.append([0.7, 0.7, 0.7])  # Default gray
                else:
                    colors.append([0.7, 0.7, 0.7])  # Default gray
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        
        if len(points) > 0:
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
            pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
            
            # Apply additional point cloud filtering
            # 1. Voxel downsampling (for uniform density)
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_downsample_size)
            
            # 2. Statistical outlier removal
            if len(pcd.points) > 20:
                try:
                    cl, ind = pcd.remove_statistical_outlier(
                        nb_neighbors=20, std_ratio=2.0)
                    pcd = pcd.select_by_index(ind)
                except Exception as e:
                    print(f"Statistical outlier removal failed: {e}")
            
            # 3. Radius outlier removal (for isolated points)
            if len(pcd.points) > 20:
                try:
                    cl, ind = pcd.remove_radius_outlier(
                        nb_points=self.outlier_min_neighbors, 
                        radius=self.outlier_radius)
                    pcd = pcd.select_by_index(ind)
                except Exception as e:
                    print(f"Radius outlier removal failed: {e}")
        
        return pcd
    
    def _fit_surfaces(self, point_cloud):
        """
        Improve stability by fitting surfaces in point cloud.
        
        Args:
            point_cloud: Input point cloud
            
        Returns:
            Stabilized point cloud with fitted planar regions
        """
        points = np.asarray(point_cloud.points)
        if len(points) < 50:
            return point_cloud
        
        # Fit plane using RANSAC
        best_plane = None
        best_inliers = []
        best_inlier_count = 0
        iterations = 50
        
        for _ in range(iterations):
            # Sample 3 random points
            sample_indices = random.sample(range(len(points)), 3)
            p1, p2, p3 = points[sample_indices]
            
            # Compute plane equation ax + by + cz + d = 0
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            if np.linalg.norm(normal) < 1e-6:
                continue
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, p1)
            
            # Count inliers
            distances = np.abs(np.dot(points, normal) + d)
            inlier_threshold = 0.02  # 2cm
            inliers = distances < inlier_threshold
            inlier_count = np.sum(inliers)
            
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_inliers = inliers
                best_plane = (normal, d)
        
        # If we found a good plane with many inliers (e.g., floor)
        inlier_ratio = best_inlier_count / len(points)
        if inlier_ratio > 0.3:  # If at least 30% of points belong to a plane
            # Create a new point cloud with adjusted positions
            inlier_points = points[best_inliers]
            normal, d = best_plane
            
            # Project inliers exactly onto the plane for stability
            for i in range(len(inlier_points)):
                # Project point onto plane
                distance = np.dot(inlier_points[i], normal) + d
                inlier_points[i] = inlier_points[i] - distance * normal
            
            # Update points in original cloud
            points_copy = points.copy()
            points_copy[best_inliers] = inlier_points
            point_cloud.points = o3d.utility.Vector3dVector(points_copy)
        
        return point_cloud
    
    def initialize_grid(self, point_cloud, padding=0.5):
        """
        Initialize or reset the 3D occupancy grid based on point cloud bounds.
        
        Args:
            point_cloud: Open3D point cloud
            padding: Additional space around the point cloud in meters
        """
        if len(point_cloud.points) == 0:
            print("Cannot initialize grid: empty point cloud")
            return
            
        points = np.asarray(point_cloud.points)
        
        if self.grid_bounds is None:
            # Compute bounds from points with padding
            min_bound = np.min(points, axis=0) - padding
            max_bound = np.max(points, axis=0) + padding
            
            self.grid_bounds = (min_bound, max_bound)
            self.min_bound = min_bound
        else:
            min_bound, max_bound = self.grid_bounds
            self.min_bound = min_bound
        
        # Calculate grid dimensions
        self.dims = np.ceil((max_bound - min_bound) / self.voxel_size).astype(int)
        
        # Create empty occupancy grid in log-odds form
        self.occupancy_log_odds = np.zeros(self.dims, dtype=np.float32)
        
        print(f"3D Grid initialized with dimensions: {self.dims}")
        print(f"Bounds: {self.grid_bounds}")
        print(f"Resolution: {self.voxel_size}m ({np.prod(self.dims)} voxels)")
    
    def _point_to_index(self, point):
        """Convert a 3D point to grid indices."""
        index = np.floor((point - self.min_bound) / self.voxel_size).astype(int)
        return index
    
    def _index_to_point(self, index):
        """Convert grid indices to 3D point (center of voxel)."""
        point = self.min_bound + (index + 0.5) * self.voxel_size
        return point
    
    def _is_in_bounds(self, index):
        """Check if indices are within grid bounds."""
        return (0 <= index[0] < self.dims[0] and 
                0 <= index[1] < self.dims[1] and 
                0 <= index[2] < self.dims[2])
    
    def _bresenham_3d(self, start, end):
        """
        Bresenham's line algorithm in 3D to get all voxels along a ray.
        
        Args:
            start: Starting point indices (x, y, z)
            end: Ending point indices (x, y, z)
            
        Returns:
            List of indices along the ray
        """
        x1, y1, z1 = start
        x2, y2, z2 = end
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dz = abs(z2 - z1)
        
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1
        sz = 1 if z2 > z1 else -1
        
        if dx >= dy and dx >= dz:
            err_1 = 2 * dy - dx
            err_2 = 2 * dz - dx
            
            indices = []
            x, y, z = x1, y1, z1
            
            for i in range(dx + 1):
                index = (x, y, z)
                if self._is_in_bounds(index):
                    indices.append(index)
                
                if err_1 > 0:
                    y += sy
                    err_1 -= 2 * dx
                
                if err_2 > 0:
                    z += sz
                    err_2 -= 2 * dx
                
                err_1 += 2 * dy
                err_2 += 2 * dz
                x += sx
        
        elif dy >= dx and dy >= dz:
            err_1 = 2 * dx - dy
            err_2 = 2 * dz - dy
            
            indices = []
            x, y, z = x1, y1, z1
            
            for i in range(dy + 1):
                index = (x, y, z)
                if self._is_in_bounds(index):
                    indices.append(index)
                
                if err_1 > 0:
                    x += sx
                    err_1 -= 2 * dy
                
                if err_2 > 0:
                    z += sz
                    err_2 -= 2 * dy
                
                err_1 += 2 * dx
                err_2 += 2 * dz
                y += sy
        
        else:  # dz >= dx and dz >= dy
            err_1 = 2 * dy - dz
            err_2 = 2 * dx - dz
            
            indices = []
            x, y, z = x1, y1, z1
            
            for i in range(dz + 1):
                index = (x, y, z)
                if self._is_in_bounds(index):
                    indices.append(index)
                
                if err_1 > 0:
                    y += sy
                    err_1 -= 2 * dz
                
                if err_2 > 0:
                    x += sx
                    err_2 -= 2 * dz
                
                err_1 += 2 * dy
                err_2 += 2 * dx
                z += sz
        
        return indices

    def update_grid(self, point_cloud, sensor_origin=np.array([0, 0, 0])):
        """
        Update the 3D occupancy grid with new point cloud data using stabilized Bayesian updates.
        
        Args:
            point_cloud: Open3D point cloud
            sensor_origin: Origin of the sensor in world coordinates
        """
        if len(point_cloud.points) == 0:
            return
            
        # Initialize grid if needed
        if self.occupancy_log_odds is None:
            self.initialize_grid(point_cloud)
            
        if self.occupancy_log_odds is None:
            return  # Initialization failed
        
        # Convert to numpy array
        points = np.asarray(point_cloud.points)
        
        # Convert sensor origin to grid indices
        origin_idx = self._point_to_index(sensor_origin)
        
        # Temporary grids for this update
        hits = np.zeros(self.dims, dtype=np.bool_)
        miss_count = np.zeros(self.dims, dtype=np.float32)
        
        # Step 1: Mark endpoint voxels as hits
        for point in points:
            idx = self._point_to_index(point)
            
            if self._is_in_bounds(idx):
                hits[tuple(idx)] = True
        
        # Step 2: Cast rays and mark free space
        # For efficiency, only process a subset of points (more sparse for ray-casting)
        ray_step = max(1, len(points) // 500)  # Adjust based on performance
        
        for i in range(0, len(points), ray_step):
            point = points[i]
            endpoint_idx = self._point_to_index(point)
            
            if self._is_in_bounds(endpoint_idx):
                # Cast ray from sensor origin to point
                ray_indices = self._bresenham_3d(origin_idx, endpoint_idx)
                
                # Mark all voxels along ray except endpoint as misses
                for idx in ray_indices[:-1]:  # Exclude endpoint
                    miss_count[idx] += 1
        
        # Step 3: Apply Bayesian update using log-odds with a conservative approach
        # More conservative updates to avoid jitter
        hit_update = self.l_occ * 0.7  # Slower increase for occupied probability
        
        # Apply hits (increase occupancy)
        self.occupancy_log_odds[hits] += hit_update
        
        # Apply misses with normalization (decrease occupancy)
        # Only update voxels that were hit by at least one ray
        miss_mask = miss_count > 0
        self.occupancy_log_odds[miss_mask] += self.l_free * np.minimum(miss_count[miss_mask], 5) / 10.0
        
        # Step 4: Apply bounds to maintain numerical stability
        self.occupancy_log_odds = np.clip(self.occupancy_log_odds, self.l_min, self.l_max)
        
        # Apply spatial smoothing to reduce jitter in the occupancy grid
        self._smooth_grid()
    
    def _smooth_grid(self):
        """Apply 3D smoothing to the occupancy grid."""
        # Only smooth uncertain regions
        uncertain = (self.occupancy_log_odds > -2.0) & (self.occupancy_log_odds < 2.0)
        
        if np.sum(uncertain) == 0:
            return
            
        # Alternative simpler smoothing for large grids
        if np.prod(self.dims) > 1000000:  # For very large grids, use simpler method
            # Apply 3D convolution with a small kernel
            kernel = np.ones((3, 3, 3)) / 27.0
            smoothed = ndimage.convolve(self.occupancy_log_odds, kernel, mode='constant', cval=0.0)
            self.occupancy_log_odds[uncertain] = smoothed[uncertain]
            return
        
        # Create a smaller grid with just the uncertain regions
        uncertain_indices = np.where(uncertain)
        
        # Apply 3D Gaussian smoothing to uncertain regions only
        # Using a small kernel size for efficiency
        kernel_size = 3
        sigma = 0.8
        
        # Create 3D Gaussian kernel
        x = np.linspace(-1, 1, kernel_size)
        y = np.linspace(-1, 1, kernel_size)
        z = np.linspace(-1, 1, kernel_size)
        xx, yy, zz = np.meshgrid(x, y, z)
        kernel = np.exp(-(xx**2 + yy**2 + zz**2) / (2 * sigma**2))
        kernel = kernel / np.sum(kernel)  # Normalize
        
        # Extract uncertain region
        padding = kernel_size // 2
        smoothed_values = np.zeros(len(uncertain_indices[0]))
        
        # For each uncertain voxel, apply smoothing
        for i in range(len(uncertain_indices[0])):
            x, y, z = uncertain_indices[0][i], uncertain_indices[1][i], uncertain_indices[2][i]
            
            # Extract neighborhood
            x_min, x_max = max(0, x - padding), min(self.dims[0], x + padding + 1)
            y_min, y_max = max(0, y - padding), min(self.dims[1], y + padding + 1)
            z_min, z_max = max(0, z - padding), min(self.dims[2], z + padding + 1)
            
            # Get values in neighborhood
            neighborhood = self.occupancy_log_odds[x_min:x_max, y_min:y_max, z_min:z_max]
            
            # Apply kernel (adjusted for boundary conditions)
            k_x_min, k_x_max = padding - (x - x_min), padding + (x_max - x)
            k_y_min, k_y_max = padding - (y - y_min), padding + (y_max - y)
            k_z_min, k_z_max = padding - (z - z_min), padding + (z_max - z)
            
            k = kernel[k_x_min:k_x_max, k_y_min:k_y_max, k_z_min:k_z_max]
            k = k / np.sum(k)  # Renormalize
            
            # Weighted average
            smoothed_values[i] = np.sum(neighborhood * k)
        
        # Update uncertain regions with smoothed values
        self.occupancy_log_odds[uncertain_indices] = smoothed_values
        
    def transform_point_cloud(self, point_cloud, transform_matrix):
        """
        Apply a transformation matrix to a point cloud.
        
        Args:
            point_cloud: Open3D point cloud
            transform_matrix: 4x4 transformation matrix
            
        Returns:
            Transformed point cloud
        """
        transformed_cloud = point_cloud.clone()
        transformed_cloud.transform(transform_matrix)
        return transformed_cloud
    
    def process_occupancy_grid_for_navigation(self, occupancy_grid=None, min_height=0.1, max_height=1.8, 
                                             erosion_radius=0.15):
        """
        Process 3D occupancy grid for 2D navigation by projecting obstacles within a height range.
        
        Args:
            occupancy_grid: 3D occupancy grid (x, y, z) -> probability, uses self.occupancy_log_odds if None
            min_height: Minimum height for obstacles (m)
            max_height: Maximum height for obstacles (m)
            erosion_radius: Safety radius around obstacles (m)
            
        Returns:
            2D grid with free (0), occupied (100), and unknown (-1) cells
        """
        if occupancy_grid is None:
            occupancy_grid = self.occupancy_log_odds
            
        # Convert log-odds to probabilities
        def log_odds_to_prob(l):
            return 1 - 1 / (1 + np.exp(l))
            
        occupancy_probs = log_odds_to_prob(occupancy_grid)
        
        # Extract coordinates of occupied voxels
        occupied_coords = np.array(np.where(occupancy_probs > 0.5)).T
        
        if len(occupied_coords) == 0:
            return np.ones((1, 1), dtype=np.int8) * -1
        
        # Project obstacles to 2D grid
        # First, determine the x-y dimensions of the 2D grid
        x_dim, y_dim, z_dim = occupancy_grid.shape
        nav_grid = np.ones((x_dim, y_dim), dtype=np.int8) * -1  # -1 = unknown
        
        # Mark free and occupied areas
        for x in range(x_dim):
            for y in range(y_dim):
                # Check if any voxel in the column is occupied within the height range
                column = occupancy_probs[x, y, :]
                
                # Get height indices corresponding to min/max height
                z_min = max(0, int(min_height / self.voxel_size))
                z_max = min(z_dim, int(max_height / self.voxel_size))
                
                # Check if column has any obstacle in the height range
                if np.any(column[z_min:z_max] > 0.5):
                    nav_grid[x, y] = 100  # Occupied
                elif np.any(column > 0):
                    # Some voxel in column is known, but not an obstacle in height range
                    nav_grid[x, y] = 0  # Free
        
        # Apply safety erosion to occupied areas (dilate obstacles)
        if erosion_radius > 0:
            # Create kernel for dilation
            kernel_radius = max(1, int(erosion_radius / self.voxel_size))
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_radius*2+1, kernel_radius*2+1))
            
            # Only apply dilation to known areas (keep unknown as unknown)
            known_mask = (nav_grid >= 0).astype(np.uint8)
            occupied_mask = (nav_grid == 100).astype(np.uint8)
            
            # Dilate occupied areas
            dilated = cv2.dilate(occupied_mask, kernel, iterations=1)
            
            # Apply dilation only to known areas
            nav_grid = np.where(known_mask > 0, dilated * 100, nav_grid)
        
        return nav_grid