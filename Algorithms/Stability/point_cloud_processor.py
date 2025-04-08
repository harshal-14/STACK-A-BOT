import numpy as np
import open3d as o3d
import os

def load_point_cloud(file_path, fix_orientation=True):
    """
    Load a point cloud from a PLY file.
    
    Args:
        file_path (str): Path to the PLY file
        fix_orientation (bool): Whether to fix upside-down orientation
        
    Returns:
        o3d.geometry.PointCloud: Loaded point cloud
    """
    print(f"Loading point cloud from {file_path}")
    pcd = o3d.io.read_point_cloud(file_path)
    
    if not pcd.has_points():
        raise ValueError("Point cloud has no points!")
    
    if fix_orientation:
        # Create a rotation matrix to flip around X axis (common fix for upside-down point clouds)
        R = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        pcd.rotate(R, center=(0, 0, 0))
        print("Applied orientation fix to correct upside-down point cloud")
    
    print(f"Loaded point cloud with {len(pcd.points)} points.")
    return pcd

def preprocess_point_cloud(pcd, voxel_size=0.01):
    """
    Preprocess the point cloud by downsampling and estimating normals.
    
    Args:
        pcd (o3d.geometry.PointCloud): Input point cloud
        voxel_size (float): Voxel size for downsampling
        
    Returns:
        o3d.geometry.PointCloud: Processed point cloud
    """
    print("Preprocessing point cloud...")
    
    # Statistical outlier removal to clean the point cloud
    print("Removing outliers...")
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)
    print(f"After outlier removal: {len(pcd.points)} points")
    
    # Use a larger voxel size to prevent excessive downsampling
    # For large point clouds (>100k points), adjust voxel size dynamically, helpful for faster processing
    # and to avoid excessive downsampling
    #TODO: Adjust heuristically based on point cloud density
    if len(pcd.points) > 100000:
        # Adjust voxel size based on point cloud density
        # Calculate bounding box volume
        bbox = pcd.get_axis_aligned_bounding_box()
        bbox_volume = np.prod(bbox.get_extent())
        point_density = len(pcd.points) / bbox_volume
        
        # Adjust voxel size to target ~10000 points after downsampling
        target_points = 10000
        adjusted_voxel_size = max(voxel_size, (len(pcd.points) / target_points / point_density) ** (1/3))
        print(f"Adjusted voxel size to {adjusted_voxel_size:.5f} based on point cloud density")
        voxel_size = adjusted_voxel_size

    downsampled = pcd.voxel_down_sample(voxel_size)

    if not downsampled.has_normals():
        print("Estimating normals...")
        downsampled.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=50)
        )
        downsampled.normalize_normals()
    else:
        # Re-estimate normals after orientation fix
        print("Re-normalizing existing normals...")
        downsampled.normalize_normals()
    
    print(f"Downsampled to {len(downsampled.points)} points.")
    return downsampled

def segment_horizontal_surfaces(pcd, angle_threshold=30, store_rotation_info=True):
    """
    Segment horizontal surfaces from the point cloud.
    
    Args:
        pcd (o3d.geometry.PointCloud): Processed point cloud with normals
        angle_threshold (float): Maximum angle deviation from vertical (in degrees)
        store_rotation_info (bool): Whether to store rotation information
        
    Returns:
        tuple: (horizontal_pcd, rotation_info)
            - horizontal_pcd (o3d.geometry.PointCloud): Point cloud with only horizontal surface points
            - rotation_info (dict): Information about the rotation applied
    """
    print("Segmenting horizontal surfaces...")
    print(f"Using angle threshold of {angle_threshold} degrees")

    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)

    # Save the point cloud with normals for inspection
    normals_pcd = o3d.geometry.PointCloud()
    normals_pcd.points = o3d.utility.Vector3dVector(points)
    normals_pcd.normals = o3d.utility.Vector3dVector(normals)
    o3d.io.write_point_cloud("normals.ply", normals_pcd)
    print("Saved point cloud with normals to 'normals.ply' for inspection")
    
    # Initialize rotation information
    rotation_info = {
        'applied_rotation': False,
        'rotation_matrix': np.eye(3),  # Default to identity matrix (no rotation)
        'angle_threshold': angle_threshold
    }
    
    # Find points with normals close to vertical (0, 0, 1)
    vertical_up = np.array([0, 0, 1])
    vertical_down = np.array([0, 0, -1])
    
    dot_products_up = np.abs(np.dot(normals, vertical_up))
    dot_products_down = np.abs(np.dot(normals, vertical_down))
    
    # Use the maximum dot product (closest match to either up or down)
    dot_products = np.maximum(dot_products_up, dot_products_down)
    
    # Convert angle threshold to dot product threshold
    cos_threshold = np.cos(np.radians(angle_threshold))
    
    # Filter points based on the dot product threshold
    horizontal_mask = dot_products > cos_threshold
    horizontal_points = points[horizontal_mask]
    horizontal_normals = normals[horizontal_mask]
    
    # Create a new point cloud with the horizontal points
    horizontal_pcd = o3d.geometry.PointCloud()
    horizontal_pcd.points = o3d.utility.Vector3dVector(horizontal_points)
    horizontal_pcd.normals = o3d.utility.Vector3dVector(horizontal_normals)

    o3d.io.write_point_cloud("horizontal_surfaces.ply", horizontal_pcd)
    
    print(f"Found {len(horizontal_pcd.points)} horizontal surface points.")
    
    # If very few horizontal points found, try plane segmentation as an alternative
    if len(horizontal_pcd.points) < 100:
        print("Few horizontal points found based on normals. Trying plane segmentation...")
        horizontal_pcd, plane_rotation = segment_planes_as_horizontal(pcd)
        
        if plane_rotation is not None:
            rotation_info['applied_rotation'] = True
            rotation_info['rotation_matrix'] = plane_rotation
    
    # Store the rotation information in a JSON file for later use
    if store_rotation_info:
        import json
        with open('rotation_info.json', 'w') as f:
            # Convert numpy array to list for JSON serialization
            rotation_info_json = rotation_info.copy()
            rotation_info_json['rotation_matrix'] = rotation_info['rotation_matrix'].tolist()
            json.dump(rotation_info_json, f, indent=2)
        print("Saved rotation information to 'rotation_info.json'")
    
    return horizontal_pcd, rotation_info
    
    return horizontal_pcd

def segment_planes_as_horizontal(pcd, distance_threshold=0.02, ransac_n=3, num_iterations=2000):
    """
    Use RANSAC plane segmentation to find horizontal surfaces.
    
    Args:
        pcd (o3d.geometry.PointCloud): Input point cloud
        distance_threshold (float): Maximum distance a point can be from the plane model
        ransac_n (int): Number of points to sample for each RANSAC iteration
        num_iterations (int): Number of RANSAC iterations
        
    Returns:
        tuple: (horizontal_pcd, rotation_matrix)
            - horizontal_pcd (o3d.geometry.PointCloud): Point cloud with horizontal surface points
            - rotation_matrix (numpy.ndarray): Rotation matrix used to align planes horizontally
    """
    print("Segmenting horizontal surfaces using RANSAC...")
    pcd_copy = o3d.geometry.PointCloud(pcd)
    
    horizontal_points = []
    horizontal_normals = []
    remaining_points = np.asarray(pcd_copy.points)
    
    # Default rotation matrix (identity = no rotation)
    rotation_matrix = None
    
    # Try to find up to 3 largest planes
    for i in range(3):
        if len(pcd_copy.points) < 10:  # Stop if too few points remain
            break
            
        # Find a plane using RANSAC, returning the model and inliers
        plane_model, inliers = pcd_copy.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations
        )
        
        if len(inliers) < 10:  # Skip if too few inliers
            continue
            
        # Extract the plane normal [a, b, c, d] where ax + by + cz + d = 0
        a, b, c, d = plane_model
        plane_normal = np.array([a, b, c])
        
        # Check if this is a horizontal plane (normal close to up/down)
        vertical_up = np.array([0, 0, 1])
        vertical_down = np.array([0, 0, -1])
        
        dot_up = np.abs(np.dot(plane_normal, vertical_up))
        dot_down = np.abs(np.dot(plane_normal, vertical_down))
        dot_product = max(dot_up, dot_down)

        horizontal_threshold = np.cos(np.radians(30))
        
        if dot_product > horizontal_threshold:
            inlier_cloud = pcd_copy.select_by_index(inliers)
            inlier_points = np.asarray(inlier_cloud.points)
            
            # If this is the first horizontal plane found, calculate rotation to align it perfectly
            if i == 0 and rotation_matrix is None:
                # Determine which direction is closer (up or down)
                if dot_up > dot_down:
                    target_normal = vertical_up
                else:
                    target_normal = vertical_down
                
                # Calculate rotation to align this plane normal with the vertical
                # This uses the Rodrigues rotation formula to find a rotation matrix
                # that rotates plane_normal to target_normal
                plane_normal_normalized = plane_normal / np.linalg.norm(plane_normal)
                
                # Calculate the rotation axis and angle
                rotation_axis = np.cross(plane_normal_normalized, target_normal)
                
                # If rotation axis is zero (vectors are parallel), no rotation needed
                if np.linalg.norm(rotation_axis) > 1e-6:
                    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                    rotation_angle = np.arccos(np.dot(plane_normal_normalized, target_normal))
                    
                    # Rodrigues rotation formula to create rotation matrix
                    K = np.array([
                        [0, -rotation_axis[2], rotation_axis[1]],
                        [rotation_axis[2], 0, -rotation_axis[0]],
                        [-rotation_axis[1], rotation_axis[0], 0]
                    ])
                    
                    rotation_matrix = (
                        np.eye(3) + 
                        np.sin(rotation_angle) * K + 
                        (1 - np.cos(rotation_angle)) * (K @ K)
                    )
                    
                    print(f"Calculated rotation matrix to align surface horizontally:")
                    print(rotation_matrix)
                else:
                    # Vectors are already aligned, no rotation needed
                    rotation_matrix = np.eye(3)
            
            inlier_normals = np.tile(plane_normal, (len(inlier_points), 1))
            
            horizontal_points.append(inlier_points)
            horizontal_normals.append(inlier_normals)
            
            print(f"Found horizontal plane with {len(inliers)} points")
        
        pcd_copy = pcd_copy.select_by_index(inliers, invert=True)
    
    if not horizontal_points:
        print("No horizontal planes found using plane segmentation.")
        empty_pcd = o3d.geometry.PointCloud()
        return empty_pcd, None

    all_points = np.vstack(horizontal_points)
    all_normals = np.vstack(horizontal_normals)

    horizontal_pcd = o3d.geometry.PointCloud()
    horizontal_pcd.points = o3d.utility.Vector3dVector(all_points)
    horizontal_pcd.normals = o3d.utility.Vector3dVector(all_normals)

    o3d.io.write_point_cloud("horizontal_planes.ply", horizontal_pcd)
    
    print(f"Found {len(horizontal_pcd.points)} horizontal surface points using plane segmentation.")
    return horizontal_pcd, rotation_matrix

def cluster_horizontal_surfaces(pcd, eps=0.05, min_points=20):
    """
    Cluster horizontal surface points to identify separate surfaces.
    
    Args:
        pcd (o3d.geometry.PointCloud): Point cloud with horizontal surface points
        eps (float): Maximum distance between points in a cluster
        min_points (int): Minimum number of points to form a cluster
        
    Returns:
        list: List of point clouds, each representing a potential surface
    """
    print("Clustering horizontal surfaces...")
    if len(pcd.points) == 0:
        print(f"Initial point cloud has {len(pcd.points)} points") #dubugginggggg!!!!!!!!
        print("No horizontal points to cluster.")
        return []
    
    # Adjust clustering parameters based on point density
    if len(pcd.points) < 100:
        # For sparse point clouds, reduce the minimum points required
        min_points = max(3, min(min_points, len(pcd.points) // 10))
        # Increase eps to allow more distant points to form clusters
        eps = min(0.1, eps * 2)
        print(f"Adjusted clustering parameters: eps={eps}, min_points={min_points}")
    
    # Use DBSCAN clustering, 
    # Useful for identifying clusters of points in 3D space
    # and can handle noise points effectively
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))
    
    if labels.max() < 0:  # No clusters found
        print("No significant horizontal surfaces found with current parameters.")
        
        # If we have points but no clusters, try again with more aggressive parameters
        if len(pcd.points) > 10:
            print("Trying more aggressive clustering parameters...")
            return cluster_horizontal_surfaces(pcd, eps=eps*1.5, min_points=max(3, min_points//2))
        return []
    
    # Separate clusters
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    clusters = []

    colors = np.zeros((len(points), 3))
    
    for i in range(labels.max() + 1):  # For each cluster
        cluster_mask = labels == i
        cluster_points = points[cluster_mask]
        cluster_normals = normals[cluster_mask]
        
        # Randoom color to clusterted points
        random_color = np.random.random(3)
        colors[cluster_mask] = random_color
        
        # Create a new point cloud for this cluster
        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
        cluster_pcd.normals = o3d.utility.Vector3dVector(cluster_normals)
        clusters.append(cluster_pcd)

    clustered_pcd = o3d.geometry.PointCloud()
    clustered_pcd.points = o3d.utility.Vector3dVector(points)
    clustered_pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud("clustered_surfaces.ply", clustered_pcd)
    
    print(f"Found {len(clusters)} horizontal surface clusters.") #Is this correct?
    return clusters