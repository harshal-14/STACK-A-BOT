import numpy as np
import open3d as o3d
import time
import cv2
import random
from sklearn.cluster import DBSCAN

def merge_point_clouds(point_clouds, voxel_size=0.05):
    """
    Merge multiple point clouds and downsample.
    
    Args:
        point_clouds: List of Open3D point clouds
        voxel_size: Voxel size for downsampling
        
    Returns:
        Merged point cloud
    """
    if not point_clouds:
        return o3d.geometry.PointCloud()
    
    # Merge all point clouds
    merged_cloud = o3d.geometry.PointCloud()
    
    for pcd in point_clouds:
        merged_cloud += pcd
    
    # Voxel downsample to avoid redundant points
    merged_cloud = merged_cloud.voxel_down_sample(voxel_size)
    
    return merged_cloud

def filter_point_cloud(point_cloud, distance_threshold=0.05, nb_neighbors=20, std_ratio=2.0):
    """
    Apply statistical and radius outlier removal to a point cloud.
    
    Args:
        point_cloud: Open3D point cloud
        distance_threshold: Distance threshold for radius outlier removal
        nb_neighbors: Number of neighbors for statistical outlier removal
        std_ratio: Standard deviation ratio for statistical outlier removal
        
    Returns:
        Filtered point cloud
    """
    if len(point_cloud.points) < 20:
        return point_cloud
    
    filtered_cloud = point_cloud
    
    # Statistical outlier removal
    try:
        cl, ind = filtered_cloud.remove_statistical_outlier(
            nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        filtered_cloud = filtered_cloud.select_by_index(ind)
    except Exception as e:
        print(f"Statistical outlier removal failed: {e}")
    
    # Radius outlier removal
    if len(filtered_cloud.points) > 10:
        try:
            cl, ind = filtered_cloud.remove_radius_outlier(
                nb_points=5, radius=distance_threshold)
            filtered_cloud = filtered_cloud.select_by_index(ind)
        except Exception as e:
            print(f"Radius outlier removal failed: {e}")
    
    return filtered_cloud

def estimate_normals(point_cloud, radius=0.1, max_nn=30):
    """
    Estimate normals for a point cloud.
    
    Args:
        point_cloud: Open3D point cloud
        radius: Radius to search for neighboring points
        max_nn: Maximum number of neighbors to consider
        
    Returns:
        Point cloud with normals
    """
    if len(point_cloud.points) < 10:
        return point_cloud
    
    # Estimate normals
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    
    # Orient normals consistently
    point_cloud.orient_normals_consistent_tangent_plane(k=15)
    
    return point_cloud

def create_mesh_from_point_cloud(point_cloud, voxel_size=0.05, depth=9, 
                                width=0, filter_outliers=True, method='poisson'):
    """
    Create a mesh from a point cloud using Poisson surface reconstruction or Ball pivoting.
    
    Args:
        point_cloud: Open3D point cloud with normals
        voxel_size: Voxel size for downsampling
        depth: Depth parameter for Poisson reconstruction
        width: Width parameter for smoothing
        filter_outliers: Whether to filter outliers before reconstruction
        method: 'poisson' or 'ball_pivoting'
        
    Returns:
        Reconstructed mesh
    """
    if len(point_cloud.points) < 100:
        print("Not enough points for mesh reconstruction")
        return None
    
    # Preprocess: downsample and estimate normals if needed
    pcd = point_cloud.voxel_down_sample(voxel_size)
    
    if filter_outliers:
        pcd = filter_point_cloud(pcd)
    
    if not pcd.has_normals():
        pcd = estimate_normals(pcd)
    
    # Reconstruct mesh
    if method == 'poisson':
        # Poisson surface reconstruction
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=depth, width=width, scale=1.1, linear_fit=False)
        
        # Filter low-density vertices
        if len(mesh.vertices) > 0:
            density_colors = np.asarray(densities)
            density_colors = density_colors / density_colors.max()
            density_mesh = o3d.geometry.TriangleMesh()
            density_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
            density_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.triangles))
            density_mesh.vertex_colors = o3d.utility.Vector3dVector(
                np.array([[c, c, c] for c in density_colors]))
            
            # Remove low-density vertices
            vertices_to_remove = densities < np.quantile(densities, 0.1)
            mesh.remove_vertices_by_mask(vertices_to_remove)
        
    elif method == 'ball_pivoting':
        # Ball pivoting: estimate radius
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radius = max(voxel_size * 2, avg_dist * 2)
        
        # Ball pivoting surface reconstruction
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector([radius, radius * 2]))
    else:
        print(f"Unknown mesh reconstruction method: {method}")
        return None
    
    # Clean and optimize mesh
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    
    return mesh

def visualize_point_clouds_and_mesh(point_clouds, mesh=None, window_name="Point Cloud and Mesh"):
    """
    Visualize multiple point clouds and an optional mesh.
    
    Args:
        point_clouds: List of Open3D point clouds
        mesh: Optional Open3D mesh
        window_name: Name of the visualization window
    """
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name, 1024, 768)
    
    # Set rendering options
    render_opt = vis.get_render_option()
    render_opt.background_color = np.array([0.1, 0.1, 0.1])
    render_opt.point_size = 3.0
    
    # Add coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5, origin=[0, 0, 0])
    vis.add_geometry(coord_frame)
    
    # Add point clouds with different colors
    colors = [
        [1, 0, 0],  # Red
        [0, 1, 0],  # Green
        [0, 0, 1],  # Blue
        [1, 1, 0],  # Yellow
        [1, 0, 1],  # Magenta
        [0, 1, 1],  # Cyan
    ]
    
    for i, pcd in enumerate(point_clouds):
        if len(pcd.points) > 0:
            # Assign color if not already colored
            if not pcd.has_colors():
                color = colors[i % len(colors)]
                pcd.paint_uniform_color(color)
            vis.add_geometry(pcd)
    
    # Add mesh if provided
    if mesh is not None and len(mesh.vertices) > 0:
        # Color mesh if not already colored
        if not mesh.has_vertex_colors():
            mesh.paint_uniform_color([0.7, 0.7, 0.7])
        vis.add_geometry(mesh)
    
    # Run visualizer
    vis.run()
    vis.destroy_window()

def depth_to_colored_point_cloud(depth_map, color_image, intrinsic_matrix, 
                                depth_scale=1000.0, depth_trunc=3.0, depth_min=0.3):
    """
    Convert depth map and color image to colored point cloud.
    
    Args:
        depth_map: Depth map (in mm)
        color_image: RGB color image aligned with depth
        intrinsic_matrix: 3x3 camera intrinsic matrix
        depth_scale: Scale factor to convert depth to meters
        depth_trunc: Maximum depth in meters
        depth_min: Minimum depth in meters
        
    Returns:
        Colored point cloud
    """
    # Convert depth to meters
    depth_meters = depth_map.astype(np.float32) / depth_scale
    
    # Convert to Open3D format
    o3d_depth = o3d.geometry.Image(depth_meters.astype(np.float32))
    o3d_color = o3d.geometry.Image(color_image.astype(np.uint8))
    
    # Create Open3D intrinsic object
    height, width = depth_meters.shape
    fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
    cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    
    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth, depth_scale=1.0, depth_trunc=depth_trunc, 
        convert_rgb_to_intensity=False)
    
    # Create point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd, intrinsic)
    
    # Filter by depth
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    
    # Calculate depth (distance from camera)
    depths = np.sqrt(np.sum(points**2, axis=1))
    
    # Create mask for valid depths
    valid_mask = (depths > depth_min) & (depths < depth_trunc)
    
    # Filter points and colors
    pcd.points = o3d.utility.Vector3dVector(points[valid_mask])
    pcd.colors = o3d.utility.Vector3dVector(colors[valid_mask])
    
    return pcd

def crop_point_cloud_by_bounding_box(point_cloud, min_bound, max_bound):
    """
    Crop a point cloud by a bounding box.
    
    Args:
        point_cloud: Open3D point cloud
        min_bound: Minimum corner of bounding box [x_min, y_min, z_min]
        max_bound: Maximum corner of bounding box [x_max, y_max, z_max]
        
    Returns:
        Cropped point cloud
    """
    # Create a bounding box
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    
    # Crop point cloud
    cropped_cloud = point_cloud.crop(bbox)
    
    return cropped_cloud

def save_point_cloud_screenshots(point_cloud, output_path_prefix, views=None):
    """
    Save screenshots of a point cloud from multiple viewpoints.
    
    Args:
        point_cloud: Open3D point cloud
        output_path_prefix: Prefix for output file paths
        views: List of view parameters (optional)
    """
    if views is None:
        # Default views: top, front, side, perspective
        views = [
            {"eye": [0, 0, 2], "up": [0, 1, 0], "front": [0, 0, -1], "name": "front"},
            {"eye": [2, 0, 0], "up": [0, 1, 0], "front": [-1, 0, 0], "name": "side"},
            {"eye": [0, 2, 0], "up": [0, 0, 1], "front": [0, -1, 0], "name": "top"},
            {"eye": [1.5, 1.5, 1.5], "up": [0, 1, 0], "front": [-1, -1, -1], "name": "perspective"}
        ]
    
    # Create visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=1280, height=960)
    
    # Add geometry
    vis.add_geometry(point_cloud)
    
    # Set rendering options
    render_opt = vis.get_render_option()
    render_opt.background_color = np.array([0.8, 0.8, 0.8])
    render_opt.point_size = 3.0
    
    # Add coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5, origin=[0, 0, 0])
    vis.add_geometry(coord_frame)
    
    # Capture views
    for i, view in enumerate(views):
        # Get view control
        vc = vis.get_view_control()
        
        # Set view
        cam = vc.convert_to_pinhole_camera_parameters()
        eye = view["eye"]
        front = view["front"]
        up = view["up"]
        name = view["name"]
        
        # Compute view matrix from look-at
        R = np.zeros((3, 3))
        t = np.zeros(3)
        
        # Compute view matrix from look-at parameters
        front = front / np.linalg.norm(front)
        right = np.cross(front, up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, front)
        
        R[:, 0] = right
        R[:, 1] = -up
        R[:, 2] = -front
        t = -R @ np.array(eye)
        
        extrinsic = np.eye(4)
        extrinsic[:3, :3] = R
        extrinsic[:3, 3] = t
        
        cam.extrinsic = extrinsic
        vc.convert_from_pinhole_camera_parameters(cam)
        
        # Update view
        vis.poll_events()
        vis.update_renderer()
        
        # Save screenshot
        timestr = time.strftime("%Y%m%d-%H%M%S")
        output_path = f"{output_path_prefix}_{name}_{timestr}.png"
        vis.capture_screen_image(output_path, do_render=True)
        print(f"Screenshot saved to {output_path}")
    
    # Close visualizer
    vis.destroy_window()

def icp_registration(source, target, voxel_size=0.05, max_correspondence_distance=0.1, 
                    method='point_to_plane'):
    """
    Align two point clouds using ICP.
    
    Args:
        source: Source point cloud
        target: Target point cloud
        voxel_size: Voxel size for downsampling
        max_correspondence_distance: Maximum correspondence distance
        method: ICP method ('point_to_point' or 'point_to_plane')
        
    Returns:
        Transformation matrix, information matrix
    """
    if len(source.points) < 10 or len(target.points) < 10:
        return np.eye(4), np.eye(6)
    
    # Downsample point clouds
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)
    
    # Estimate normals if using point-to-plane
    if method == 'point_to_plane':
        source_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))
        target_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 2, max_nn=30))
    
    # Prepare ICP algorithm
    if method == 'point_to_plane':
        icp = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        icp = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    
    # Run ICP
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, max_correspondence_distance, np.eye(4),
        icp,
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=100))
    
    return result.transformation, result.information

def detect_planes_in_point_cloud(point_cloud, distance_threshold=0.02, ransac_n=3, 
                               num_iterations=1000, min_plane_size=100):
    """
    Detect major planes in a point cloud using RANSAC.
    
    Args:
        point_cloud: Open3D point cloud
        distance_threshold: Maximum distance from point to plane
        ransac_n: Number of points for RANSAC
        num_iterations: Number of RANSAC iterations
        min_plane_size: Minimum number of points for a valid plane
        
    Returns:
        List of plane segments (point clouds), list of plane equations
    """
    planes = []
    plane_equations = []
    
    # Make a copy of the point cloud
    remaining_cloud = point_cloud.clone()
    
    # Color the original point cloud for reference
    original_colors = np.asarray(point_cloud.colors)
    
    # Try to find multiple planes
    max_planes = 5  # Maximum number of planes to detect
    
    for i in range(max_planes):
        if len(remaining_cloud.points) < min_plane_size:
            break
            
        # Run RANSAC
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations)
        
        # Skip if too few inliers
        if len(inliers) < min_plane_size:
            break
            
        # Extract plane segment
        plane_cloud = remaining_cloud.select_by_index(inliers)
        
        # Assign color to plane (rainbow colors)
        colors = [
            [1, 0, 0],  # Red
            [0, 1, 0],  # Green
            [0, 0, 1],  # Blue
            [1, 1, 0],  # Yellow
            [1, 0, 1],  # Magenta
        ]
        plane_color = colors[i % len(colors)]
        plane_cloud.paint_uniform_color(plane_color)
        
        # Add to results
        planes.append(plane_cloud)
        plane_equations.append(plane_model)
        
        # Remove plane points from the remaining cloud
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)
    
    return planes, plane_equations

def create_voxel_heightmap(point_cloud, voxel_size=0.05, height_colormap=True):
    """
    Create a height map from a point cloud using voxel grid.
    
    Args:
        point_cloud: Open3D point cloud
        voxel_size: Size of each voxel in meters
        height_colormap: Whether to color by height
        
    Returns:
        Height map as 2D numpy array, colored point cloud for visualization
    """
    if len(point_cloud.points) < 10:
        return None, point_cloud
    
    # Get point coordinates
    points = np.asarray(point_cloud.points)
    
    # Determine grid bounds
    min_bound = np.min(points, axis=0)
    max_bound = np.max(points, axis=0)
    
    # Create 2D grid for heightmap
    x_size = int(np.ceil((max_bound[0] - min_bound[0]) / voxel_size))
    y_size = int(np.ceil((max_bound[1] - min_bound[1]) / voxel_size))
    
    # Initialize height map with NaN
    height_map = np.ones((x_size, y_size)) * np.nan
    
    # Process points
    for point in points:
        # Convert to grid coordinates
        x_idx = int((point[0] - min_bound[0]) / voxel_size)
        y_idx = int((point[1] - min_bound[1]) / voxel_size)
        
        if 0 <= x_idx < x_size and 0 <= y_idx < y_size:
            # Update with maximum height at this xy location
            if np.isnan(height_map[x_idx, y_idx]) or point[2] > height_map[x_idx, y_idx]:
                height_map[x_idx, y_idx] = point[2]
    
    # Create a colored point cloud for visualization
    vis_cloud = o3d.geometry.PointCloud()
    
    # Create points for each grid cell
    vis_points = []
    vis_colors = []
    
    # Color map for height
    min_height = np.nanmin(height_map)
    max_height = np.nanmax(height_map)
    height_range = max(0.001, max_height - min_height)
    
    for x_idx in range(x_size):
        for y_idx in range(y_size):
            if not np.isnan(height_map[x_idx, y_idx]):
                # Convert back to world coordinates
                x = min_bound[0] + (x_idx + 0.5) * voxel_size
                y = min_bound[1] + (y_idx + 0.5) * voxel_size
                z = height_map[x_idx, y_idx]
                
                vis_points.append([x, y, z])
                
                if height_colormap:
                    # Color by height
                    norm_z = (z - min_height) / height_range
                    if norm_z < 0.33:
                        # Blue to cyan
                        color = [0, norm_z * 3, 1]
                    elif norm_z < 0.66:
                        # Cyan to yellow
                        color = [(norm_z - 0.33) * 3, 1, 1 - (norm_z - 0.33) * 3]
                    else:
                        # Yellow to red
                        color = [1, 1 - (norm_z - 0.66) * 3, 0]
                else:
                    # Use grayscale
                    gray = (z - min_height) / height_range
                    color = [gray, gray, gray]
                
                vis_colors.append(color)
    
    # Create visualization point cloud
    if vis_points:
        vis_cloud.points = o3d.utility.Vector3dVector(np.array(vis_points))
        vis_cloud.colors = o3d.utility.Vector3dVector(np.array(vis_colors))
    
    return height_map, vis_cloud

def create_3d_path_visualization(path_points, height=0.1, radius=0.05, color=[0, 1, 0]):
    """
    Create a visualization of a 3D path as a series of spheres and cylinders.
    
    Args:
        path_points: List of 3D path points
        height: Height above ground for path
        radius: Radius of path segments
        color: Color of path
        
    Returns:
        Open3D geometry for path visualization
    """
    if len(path_points) < 2:
        return o3d.geometry.TriangleMesh()
    
    # Ensure path points have the right height
    adjusted_points = []
    for point in path_points:
        if len(point) == 2:
            # 2D point - add height
            adjusted_points.append([point[0], point[1], height])
        else:
            # Use specified height
            adjusted_points.append([point[0], point[1], height])
    
    path_vis = o3d.geometry.TriangleMesh()
    
    # Create a sphere at each waypoint
    for point in adjusted_points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        sphere.translate(point)
        sphere.paint_uniform_color(color)
        path_vis += sphere
    
    # Create cylinders between waypoints
    for i in range(len(adjusted_points) - 1):
        p1 = adjusted_points[i]
        p2 = adjusted_points[i + 1]
        
        # Calculate cylinder parameters
        direction = np.array(p2) - np.array(p1)
        length = np.linalg.norm(direction)
        direction = direction / length
        
        # Create transformation
        z_axis = np.array([0, 0, 1])
        if np.allclose(direction, z_axis) or np.allclose(direction, -z_axis):
            rotation = np.eye(3)
        else:
            rotation_axis = np.cross(z_axis, direction)
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            
            angle = np.arccos(np.dot(z_axis, direction))
            
            # Rodrigues formula for rotation matrix
            K = np.array([
                [0, -rotation_axis[2], rotation_axis[1]],
                [rotation_axis[2], 0, -rotation_axis[0]],
                [-rotation_axis[1], rotation_axis[0], 0]
            ])
            rotation = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        # Create cylinder
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius/2, height=length)
        
        # Apply transformation
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = p1
        
        # Move cylinder center to origin, rotate, then translate
        cylinder.translate([0, 0, -length/2])
        cylinder.transform(transform)
        cylinder.paint_uniform_color(color)
        
        path_vis += cylinder
    
    return path_vis

def detect_objects_in_point_cloud(point_cloud, min_points=50, max_points=10000, 
                                eps=0.1, min_samples=10):
    """
    Detect objects in a point cloud using DBSCAN clustering.
    
    Args:
        point_cloud: Open3D point cloud
        min_points: Minimum points in a cluster to be considered an object
        max_points: Maximum points in a cluster
        eps: DBSCAN epsilon parameter
        min_samples: DBSCAN min_samples parameter
        
    Returns:
        List of point clouds for detected objects
    """
    if len(point_cloud.points) < min_points:
        return []
    
    # Apply DBSCAN clustering
    points = np.asarray(point_cloud.points)
    colors = np.asarray(point_cloud.colors)
    
    # Use DBSCAN from scikit-learn
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    
    # Get cluster labels
    labels = clustering.labels_
    
    # Count number of clusters (excluding noise with label -1)
    n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
    
    if n_clusters == 0:
        return []
    
    # Extract clusters
    objects = []
    cluster_colors = [
        [1, 0, 0],    # Red
        [0, 1, 0],    # Green
        [0, 0, 1],    # Blue
        [1, 1, 0],    # Yellow
        [1, 0, 1],    # Magenta
        [0, 1, 1],    # Cyan
        [1, 0.5, 0],  # Orange
        [0.5, 0, 1],  # Purple
        [0, 0.5, 0],  # Dark green
        [0.5, 0.5, 1] # Light blue
    ]
    
    for cluster_idx in range(n_clusters):
        # Get points in this cluster
        cluster_mask = (labels == cluster_idx)
        cluster_size = np.sum(cluster_mask)
        
        # Filter by size
        if min_points <= cluster_size <= max_points:
            cluster_points = points[cluster_mask]
            
            # Create a new point cloud for this cluster
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)
            
            # Assign color to cluster
            cluster_cloud.paint_uniform_color(cluster_colors[cluster_idx % len(cluster_colors)])
            
            objects.append(cluster_cloud)
    
    return objects

def analyze_point_cloud_statistics(point_cloud):
    """
    Analyze basic statistics of a point cloud.
    
    Args:
        point_cloud: Open3D point cloud
        
    Returns:
        Dictionary of statistics
    """
    if len(point_cloud.points) == 0:
        return {"point_count": 0}
    
    points = np.asarray(point_cloud.points)
    
    # Basic statistics
    stats = {
        "point_count": len(points),
        "min_bound": points.min(axis=0).tolist(),
        "max_bound": points.max(axis=0).tolist(),
        "dimensions": (points.max(axis=0) - points.min(axis=0)).tolist(),
        "center": points.mean(axis=0).tolist()
    }
    
    # Calculate point density
    bbox_volume = np.prod(stats["dimensions"])
    if bbox_volume > 0:
        stats["point_density"] = stats["point_count"] / bbox_volume
    else:
        stats["point_density"] = 0
    
    # Calculate nearest neighbor statistics
    pcd_tree = o3d.geometry.KDTreeFlann(point_cloud)
    
    nn_distances = []
    sample_count = min(1000, len(points))
    sample_indices = np.random.choice(len(points), sample_count, replace=False)
    
    for idx in sample_indices:
        _, indices, distances = pcd_tree.search_knn_vector_3d(point_cloud.points[idx], 2)
        if len(distances) > 1:  # Exclude self
            nn_distances.append(np.sqrt(distances[1]))
    
    if nn_distances:
        stats["avg_nearest_neighbor_distance"] = np.mean(nn_distances)
        stats["min_nearest_neighbor_distance"] = np.min(nn_distances)
        stats["max_nearest_neighbor_distance"] = np.max(nn_distances)
    
    return stats