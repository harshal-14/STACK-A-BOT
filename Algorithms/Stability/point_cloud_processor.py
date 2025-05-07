import numpy as np
import open3d as o3d

def load_point_cloud(file_path, fix_orientation=True):
    """Load a point cloud from a PLY file with basic orientation fix"""
    print(f"Loading point cloud from {file_path}")
    pcd = o3d.io.read_point_cloud(file_path)
    
    if not pcd.has_points():
        raise ValueError("Point cloud has no points!")
    
    if fix_orientation:
        # Create a rotation matrix to flip around X axis
        R = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        pcd.rotate(R, center=(0, 0, 0))
    
    print(f"Loaded point cloud with {len(pcd.points)} points.")
    return pcd

def preprocess_point_cloud(pcd, voxel_size=0.005):
    """Clean and prepare point cloud for analysis"""
    print("Preprocessing point cloud...")
    
    # Statistical outlier removal to clean the point cloud
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.5)
    pcd = pcd.select_by_index(ind)
    print(f"After outlier removal: {len(pcd.points)} points")
    
    # Dynamic voxel size based on point cloud density
    if len(pcd.points) < 5000:
        bbox = pcd.get_axis_aligned_bounding_box()
        bbox_volume = np.prod(bbox.get_extent())
        point_density = len(pcd.points) / max(bbox_volume, 0.001)
        target_points = 100000
        voxel_size = max(voxel_size, (len(pcd.points) / target_points / point_density) ** (1/3))
        print(f"Adjusted voxel size to {voxel_size:.5f} based on point cloud density")

    downsampled = pcd.voxel_down_sample(voxel_size)

    # Estimate normals if not present
    if not downsampled.has_normals():
        print("Estimating normals...")
        downsampled.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=50)
        )
    downsampled.normalize_normals()
    
    print(f"Downsampled to {len(downsampled.points)} points.")
    return downsampled

def align_point_cloud(pcd):
    """Align point cloud so the main plane is horizontal and Z-up"""
    # Find dominant plane (pallet/table surface)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.01,
        ransac_n=4,
        num_iterations=3000
    )
    a, b, c, d = plane_model  # Plane equation: ax + by + cz + d = 0
    
    # Create rotation to align this plane with XY plane (Z-up)
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)  # Normalize
    
    # Calculate rotation to align normal with [0,0,1]
    z_axis = np.array([0, 0, 1])
    rotation_axis = np.cross(normal, z_axis)
    
    if np.linalg.norm(rotation_axis) > 1e-6:
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        angle = np.arccos(np.dot(normal, z_axis))
        
        # Create rotation matrix using Rodriguez formula
        K = np.array([
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0]
        ])
        rotation = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        # Apply rotation
        aligned_pcd = o3d.geometry.PointCloud(pcd)
        aligned_pcd.rotate(rotation, center=[0, 0, 0])
        
        print(f"Aligned point cloud with rotation angle {np.degrees(angle):.2f}°")
        return aligned_pcd
    
    return pcd  # Already aligned