"""
Advanced point cloud segmentation module.
"""
import open3d as o3d
import numpy as np

def segment_point_cloud(points, voxel_size=0.02):
    """
    Advanced point cloud segmentation to isolate objects from background.
    
    This function uses a combination of:
    1. Plane segmentation (to remove ground plane)
    2. Euclidean clustering with optimized parameters
    3. Region growing segmentation for smoother results
    
    Args:
        points: Numpy array of points or Open3D point cloud
        voxel_size: Size parameter for processing (smaller = more detailed but slower)
    
    Returns:
        List of segmented point clouds (one per object)
    """
    print("Starting advanced point cloud segmentation...")
    
    # Convert to Open3D point cloud if needed
    if isinstance(points, np.ndarray):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
    else:
        pcd = points
    
    # Downsample the point cloud for faster processing
    print("Downsampling point cloud...")
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    
    # Estimate normals (needed for plane segmentation and region growing)
    print("Estimating normals...")
    down_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size*2, max_nn=30))
    
    # 1. Detect and remove the ground plane
    print("Detecting ground plane...")
    plane_model, inliers = down_pcd.segment_plane(
        distance_threshold=voxel_size*2,
        ransac_n=3,
        num_iterations=1000)
    
    # Extract ground plane and objects
    ground_cloud = down_pcd.select_by_index(inliers)
    object_cloud = down_pcd.select_by_index(inliers, invert=True)
    
    # Color the ground and objects differently for visualization
    ground_cloud.paint_uniform_color([0.8, 0.8, 0.8])  # Light gray for ground
    
    # If no objects found above ground, return just the point cloud
    if len(object_cloud.points) < 10:
        print("No objects found above ground plane.")
        return [pcd]
    
    # 2. Euclidean clustering to separate distinct objects
    print("Clustering objects...")
    
    # First try Open3D's native DBSCAN clustering
    labels = np.array(object_cloud.cluster_dbscan(
        eps=voxel_size*10,
        min_points=10))
    
    max_label = labels.max()
    print(f"Found {max_label + 1} clusters")
    
    # If insufficient clusters found, try a more sophisticated approach
    if max_label < 0:
        print("No clusters found, using full object cloud.")
        clusters = [object_cloud]
    else:
        # Create a list of point clouds for each cluster
        clusters = []
        for i in range(max_label + 1):
            cluster_indices = np.where(labels == i)[0]
            
            # Only keep clusters with enough points (to filter noise)
            if len(cluster_indices) >= 50:
                cluster = object_cloud.select_by_index(cluster_indices)
                
                # Apply statistical outlier removal to clean the cluster
                try:
                    cl, ind = cluster.remove_statistical_outlier(
                        nb_neighbors=20, std_ratio=2.0)
                    cluster = cluster.select_by_index(ind)
                except:
                    pass  # Keep original if outlier removal fails
                
                clusters.append(cluster)
    
    # If no valid clusters, return the original object cloud
    if len(clusters) == 0:
        print("No valid clusters found, using entire object point cloud.")
        # Fall back to a different approach - trying region growing segmentation
        try:
            print("Attempting region growing segmentation...")
            # This is a simplified version as Open3D doesn't have direct region growing
            # We'll approximate it by normal-based smoothing and clustering
            
            # Smooth normals
            for _ in range(3):
                # Get neighbors list
                kdtree = o3d.geometry.KDTreeFlann(object_cloud)
                normals = np.asarray(object_cloud.normals)
                points = np.asarray(object_cloud.points)
                
                new_normals = np.copy(normals)
                for i in range(len(points)):
                    [_, idx, _] = kdtree.search_knn_vector_3d(points[i], 10)
                    neighborhood_normals = normals[idx]
                    # Average normal direction
                    avg_normal = np.mean(neighborhood_normals, axis=0)
                    if np.linalg.norm(avg_normal) > 0:
                        new_normals[i] = avg_normal / np.linalg.norm(avg_normal)
                
                object_cloud.normals = o3d.utility.Vector3dVector(new_normals)
            
            # Group by similar normals using dot product
            normals = np.asarray(object_cloud.normals)
            points = np.asarray(object_cloud.points)
            
            # Manual region growing-like algorithm
            unprocessed = np.ones(len(points), dtype=bool)
            regions = []
            
            while np.any(unprocessed):
                # Start with first unprocessed point
                seed_idx = np.where(unprocessed)[0][0]
                seed_normal = normals[seed_idx]
                
                # Grow region
                region = [seed_idx]
                unprocessed[seed_idx] = False
                
                for idx in region:
                    # Find neighbors (simple K-nearest)
                    [_, neighbors, _] = kdtree.search_knn_vector_3d(points[idx], 20)
                    
                    for neighbor in neighbors:
                        if unprocessed[neighbor]:
                            # Check normal similarity
                            similarity = np.dot(normals[neighbor], seed_normal)
                            if similarity > 0.8:  # Cos similarity threshold
                                region.append(neighbor)
                                unprocessed[neighbor] = False
                
                if len(region) > 50:  # Only keep reasonably sized regions
                    region_cloud = object_cloud.select_by_index(region)
                    regions.append(region_cloud)
            
            if len(regions) > 0:
                clusters = regions
            else:
                clusters = [object_cloud]
                
        except Exception as e:
            print(f"Region growing failed: {e}")
            clusters = [object_cloud]
    
    # 3. Add the ground plane as a separate segment
    all_segments = clusters + [ground_cloud]
    
    # Colorize each cluster uniquely for visualization
    colors = [
        [1, 0, 0],    # Red
        [0, 1, 0],    # Green
        [0, 0, 1],    # Blue
        [1, 1, 0],    # Yellow
        [1, 0, 1],    # Magenta
        [0, 1, 1],    # Cyan
        [1, 0.5, 0],  # Orange
        [0.5, 0, 1],  # Purple
        [0, 0.5, 0]   # Dark green
    ]
    
    for i, cluster in enumerate(clusters):
        color_idx = i % len(colors)
        cluster.paint_uniform_color(colors[color_idx])
    
    print(f"Segmentation complete. Found {len(clusters)} object segments plus ground plane.")
    return all_segments

def visualize_segments(segments):
    """
    Visualize the segmented point clouds.
    
    Args:
        segments: List of Open3D point clouds (one per segment)
    """
    print(f"Visualizing {len(segments)} segments...")
    o3d.visualization.draw_geometries(segments)

if __name__ == "__main__":
    # Example usage as a standalone script
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python segment_point_cloud.py point_cloud.ply")
        sys.exit(1)
    
    # Load point cloud
    print(f"Loading point cloud from {sys.argv[1]}...")
    try:
        pcd = o3d.io.read_point_cloud(sys.argv[1])
    except Exception as e:
        print(f"Error loading point cloud: {e}")
        sys.exit(1)
    
    # Perform segmentation
    segments = segment_point_cloud(pcd)
    
    # Visualize results
    visualize_segments(segments)
    
    # Save segments
    for i, segment in enumerate(segments):
        output_path = f"segment_{i}.ply"
        o3d.io.write_point_cloud(output_path, segment)
        print(f"Saved segment {i} to {output_path}")