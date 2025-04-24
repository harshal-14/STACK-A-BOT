import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull
from sklearn.decomposition import PCA

def calculate_surface_area(points):
    """Calculate approximate surface area using 2D convex hull"""
    if len(points) < 3:
        return 0.0
        
    # Project points to XY plane
    xy_points = points[:, :2]
    
    try:
        hull = ConvexHull(xy_points)
        return hull.volume  # 2D "volume" is actually area
    except Exception as e:
        print(f"Error calculating surface area: {e}")
        return 0.0

def analyze_surface_flatness(points):
    """Analyze surface flatness using PCA"""
    if len(points) < 3:
        return 0.0
        
    pca = PCA(n_components=3)
    pca.fit(points)
    
    # Smallest eigenvalue indicates variation perpendicular to the surface
    eigenvalues = pca.explained_variance_
    flatness = 1.0 - (eigenvalues[2] / max(np.sum(eigenvalues), 1e-6))
    
    return flatness

def evaluate_stacking_stability(bottom_box, top_box, contact_threshold=0.7):
    """Evaluate stability of stacking one box on top of another"""
    # Extract dimensions
    bottom_dims = bottom_box['dimensions']
    top_dims = top_box['dimensions']
    
    # Calculate the contact area (intersection of footprints)
    overlap_width = min(bottom_dims[0], top_dims[0])
    overlap_depth = min(bottom_dims[1], top_dims[1])
    
    # Calculate overlap ratio (how much of the top box is supported)
    top_area = top_dims[0] * top_dims[1]
    overlap_area = overlap_width * overlap_depth
    
    support_ratio = overlap_area / max(top_area, 1e-6)
    
    # Simple stability check - enough support area
    stable = support_ratio >= contact_threshold
    
    # Center of gravity stability check
    if stable:
        # Check if the center of the top box is over the bottom box
        bottom_pos = bottom_box['position']
        top_pos = top_box['position']
        
        # X-axis check
        if (top_pos[0] < bottom_pos[0] - bottom_dims[0]/2 + top_dims[0]/4 or
            top_pos[0] > bottom_pos[0] + bottom_dims[0]/2 - top_dims[0]/4):
            stable = False
            
        # Y-axis check
        if (top_pos[1] < bottom_pos[1] - bottom_dims[1]/2 + top_dims[1]/4 or
            top_pos[1] > bottom_pos[1] + bottom_dims[1]/2 - top_dims[1]/4):
            stable = False
    
    return {
        'stable': stable,
        'support_ratio': support_ratio,
        'score': support_ratio if stable else 0.0
    }

def check_collision(self, new_box, existing_boxes, tolerance=0.002):
    """Enhanced collision detection with better debugging"""
    new_pos = np.array(new_box['position'])
    new_dims = np.array(new_box['dimensions'])
    
    # Calculate new box bounds with tolerance
    new_min = new_pos - new_dims/2 - tolerance
    new_max = new_pos + new_dims/2 + tolerance
    
    for i, existing_box in enumerate(existing_boxes):
        # Skip comparison with self
        if existing_box is new_box:
            continue
            
        ex_pos = np.array(existing_box['position'])
        ex_dims = np.array(existing_box['dimensions'])
        
        # Calculate existing box bounds
        ex_min = ex_pos - ex_dims/2 - tolerance
        ex_max = ex_pos + ex_dims/2 + tolerance
        
        # Check for overlap in all three dimensions
        overlap_x = new_min[0] <= ex_max[0] and ex_min[0] <= new_max[0]
        overlap_y = new_min[1] <= ex_max[1] and ex_min[1] <= new_max[1]
        overlap_z = new_min[2] <= ex_max[2] and ex_min[2] <= new_max[2]
        
        # If overlapping in all dimensions, we have a collision
        if overlap_x and overlap_y and overlap_z:
            print(f"COLLISION DETECTED with box {i}:")
            print(f"  New box: pos={new_pos}, dims={new_dims}")
            print(f"  Box {i}: pos={ex_pos}, dims={ex_dims}")
            return True
    
    return False

def get_box_top_surface(box):
    """Get the position and dimensions of a box's top surface"""
    position = box['position'].copy()
    dimensions = box['dimensions']
    
    # Top surface is at half height above the center
    position[2] += dimensions[2] / 2
    
    surface = {
        'position': position,
        'dimensions': [dimensions[0], dimensions[1], 0],  # Zero height
        'parent_box': box,
        'type': 'top'
    }
    
    return surface

def get_box_side_surfaces(box):
    """Get the four side surfaces of a box for adjacent placement"""
    position = box['position']
    dimensions = box['dimensions']
    half_width = dimensions[0] / 2
    half_depth = dimensions[1] / 2
    half_height = dimensions[2] / 2
    
    # Create the four side surfaces
    sides = []
    
    # X+ side (right)
    right_pos = position.copy()
    right_pos[0] += half_width
    sides.append({
        'position': right_pos,
        'dimensions': [0, dimensions[1], dimensions[2]],
        'parent_box': box,
        'type': 'right'
    })
    
    # X- side (left)
    left_pos = position.copy()
    left_pos[0] -= half_width
    sides.append({
        'position': left_pos,
        'dimensions': [0, dimensions[1], dimensions[2]],
        'parent_box': box,
        'type': 'left'
    })
    
    # Y+ side (front)
    front_pos = position.copy()
    front_pos[1] += half_depth
    sides.append({
        'position': front_pos,
        'dimensions': [dimensions[0], 0, dimensions[2]],
        'parent_box': box,
        'type': 'front'
    })
    
    # Y- side (back)
    back_pos = position.copy()
    back_pos[1] -= half_depth
    sides.append({
        'position': back_pos,
        'dimensions': [dimensions[0], 0, dimensions[2]],
        'parent_box': box,
        'type': 'back'
    })
    
    return sides



def detect_existing_boxes_from_point_cloud(self):
    """Detect existing boxes in the scene from point cloud"""
    if not hasattr(self, 'objects_pcd') or len(self.objects_pcd.points) == 0:
        print("No objects to detect as boxes")
        return []
        
    existing_boxes = []
    
    # Try DBSCAN clustering to identify separate objects
    labels = np.array(self.objects_pcd.cluster_dbscan(eps=0.02, min_points=5))
    
    if labels.max() < 0:
        print("No clusters found in object point cloud")
        return []
    
    points = np.asarray(self.objects_pcd.points)
    
    for i in range(labels.max() + 1):
        cluster_points = points[labels == i]
        
        if len(cluster_points) < 10:  # Skip very small clusters
            continue
            
        # Calculate bounding box
        min_bound = np.min(cluster_points, axis=0)
        max_bound = np.max(cluster_points, axis=0)
        dimensions = max_bound - min_bound
        center = (min_bound + max_bound) / 2
        
        # Skip if too small
        if np.any(dimensions < 0.005):  # Minimum 5mm in each dimension
            continue
            
        # Skip if too large
        if np.any(dimensions > 0.2):  # Maximum 20cm in each dimension
            continue
            
        existing_boxes.append({
            'position': center,
            'dimensions': dimensions,
            'detected': True,  # Flag to indicate this was detected, not placed
            'placement_type': 'detected'
        })
        
        print(f"Detected box at {center} with dimensions {dimensions}")
    
    return existing_boxes

def match_box_scale_to_point_cloud(self, box_sizes):
    """Match box dimensions to point cloud scale - fixed implementation"""
    # We need a direct conversion factor: 1 cm = 0.01 m
    cm_to_m = 0.01
    
    scaled_sizes = []
    for size in box_sizes:
        # Convert directly from cm to meters
        scaled_size = tuple(dim * cm_to_m for dim in size)
        scaled_sizes.append(scaled_size)
    
    print(f"Box sizes converted from cm to meters:")
    print(f"Original (cm): {box_sizes}")
    print(f"Scaled (m): {scaled_sizes}")
    
    return scaled_sizes

def segment_pallet_from_table(self, pcd, visualize=True):
    """Robust pallet segmentation for very small height differences"""
    print("Performing robust pallet and table segmentation...")
    
    # Get all points
    points = np.asarray(pcd.points)
    
    # Step 1: Find the dominant plane using RANSAC
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=0.002,  # Use smaller threshold for precise segmentation
        ransac_n=3,
        num_iterations=2000
    )
    
    # Extract table points
    table_pcd = pcd.select_by_index(inliers)
    remaining_pcd = pcd.select_by_index(inliers, invert=True)
    
    # Step 2: If there are still many points, try to find pallet plane
    if len(remaining_pcd.points) > 100:
        try:
            # Find second dominant plane (pallet)
            pallet_model, pallet_inliers = remaining_pcd.segment_plane(
                distance_threshold=0.002,
                ransac_n=3,
                num_iterations=1000
            )
            
            pallet_pcd = remaining_pcd.select_by_index(pallet_inliers)
            objects_pcd = remaining_pcd.select_by_index(pallet_inliers, invert=True)
            
            # Compare heights
            table_height = np.mean(np.asarray(table_pcd.points)[:, 2])
            pallet_height = np.mean(np.asarray(pallet_pcd.points)[:, 2])
            height_diff = abs(pallet_height - table_height)
            
            # If heights are too similar, use a different approach
            if height_diff < 0.003:  # Less than 3mm difference
                print(f"Table-pallet height difference too small ({height_diff:.6f}m)")
                print("Using connected component analysis to separate pallet")
                
                # Try connected component analysis
                with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                    labels = np.array(remaining_pcd.cluster_dbscan(eps=0.02, min_points=10))
                
                if labels.max() >= 0:
                    # Find the largest cluster (likely the pallet)
                    largest_cluster_idx = np.argmax([np.sum(labels == i) for i in range(labels.max() + 1)])
                    pallet_mask = labels == largest_cluster_idx
                    
                    # Extract pallet and objects
                    pallet_points = np.asarray(remaining_pcd.points)[pallet_mask]
                    object_points = np.asarray(remaining_pcd.points)[~pallet_mask & (labels >= 0)]
                    
                    # Create new point clouds
                    pallet_pcd = o3d.geometry.PointCloud()
                    pallet_pcd.points = o3d.utility.Vector3dVector(pallet_points)
                    
                    objects_pcd = o3d.geometry.PointCloud()
                    objects_pcd.points = o3d.utility.Vector3dVector(object_points)
            
            # Colorize for visualization
            table_pcd.paint_uniform_color([0.7, 0.7, 0.7])  # Light gray
            pallet_pcd.paint_uniform_color([0.0, 0.8, 0.8])  # Cyan
            objects_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # Red
            
            print(f"Table points: {len(table_pcd.points)}")
            print(f"Pallet points: {len(pallet_pcd.points)}")
            print(f"Object points: {len(objects_pcd.points)}")
            
            # Store the height difference for box placement
            self.pallet_height = np.mean(np.asarray(pallet_pcd.points)[:, 2])
            self.table_height = np.mean(np.asarray(table_pcd.points)[:, 2])
            print(f"Pallet is {self.pallet_height - self.table_height:.6f}m above the table")
            
            # Visualize the segmentation if requested
            if visualize:
                o3d.visualization.draw_geometries(
                    [table_pcd, pallet_pcd, objects_pcd],
                    window_name="Table-Pallet-Objects Segmentation",
                    width=800,
                    height=600
                )
            
            return table_pcd, pallet_pcd, objects_pcd
            
        except Exception as e:
            print(f"Error in pallet segmentation: {e}")
            
    # Fallback: use table as base and remaining points as objects
    print("Falling back to basic table-object segmentation")
    table_pcd.paint_uniform_color([0.7, 0.7, 0.7])  # Light gray
    remaining_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # Red
    
    # Create an empty pallet
    pallet_pcd = o3d.geometry.PointCloud()
    
    return table_pcd, pallet_pcd, remaining_pcd

def create_box_marker(position, dimensions, color=[1, 0, 0]):
    """Create a box wireframe marker for visualization"""
    half_width = dimensions[0] / 2
    half_depth = dimensions[1] / 2
    half_height = dimensions[2] / 2
    
    vertices = [
        # Bottom face
        [position[0] - half_width, position[1] - half_depth, position[2] - half_height],
        [position[0] + half_width, position[1] - half_depth, position[2] - half_height],
        [position[0] + half_width, position[1] + half_depth, position[2] - half_height],
        [position[0] - half_width, position[1] + half_depth, position[2] - half_height],
        # Top face
        [position[0] - half_width, position[1] - half_depth, position[2] + half_height],
        [position[0] + half_width, position[1] - half_depth, position[2] + half_height],
        [position[0] + half_width, position[1] + half_depth, position[2] + half_height],
        [position[0] - half_width, position[1] + half_depth, position[2] + half_height]
    ]
    
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
        [0, 4], [1, 5], [2, 6], [3, 7]   # Connecting edges
    ]
    
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    
    # Set color
    colors = [color for _ in range(len(lines))]
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set

def isolate_pallet_for_stacking(self):
    """Isolate just the pallet surface and objects for stacking analysis"""
    if not hasattr(self, 'pallet_pcd') or len(self.pallet_pcd.points) < 50:
        print("Warning: No pallet detected or pallet has too few points")
        return False
        
    # Get pallet bounding box
    pallet_bbox = self.pallet_pcd.get_axis_aligned_bounding_box()
    min_bound = np.array(pallet_bbox.min_bound.copy())  # Create a copy
    max_bound = np.array(pallet_bbox.max_bound.copy())  # Create a copy
    
    # Add a small margin above the pallet to capture objects on it
    min_bound[2] = min_bound[2] - 0.01  # 1cm below pallet
    max_bound[2] = max_bound[2] + 0.10  # 10cm above pallet
    
    # Create a cropped version of the point cloud including only the pallet region
    points = np.asarray(self.processed_pcd.points)
    
    # Find points within the pallet bounding box
    mask = np.all((points >= min_bound) & (points <= max_bound), axis=1)
    pallet_region_points = points[mask]
    
    # Create a new point cloud with just these points
    pallet_region_pcd = o3d.geometry.PointCloud()
    pallet_region_pcd.points = o3d.utility.Vector3dVector(pallet_region_points)
    
    # Re-segment within this region to get refined pallet surface and objects
    print(f"Isolated pallet region with {len(pallet_region_points)} points")
    
    # Re-estimate normals for the cropped point cloud
    pallet_region_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
    )
    
    # Store the cropped point cloud for stacking
    self.pallet_region_pcd = pallet_region_pcd
    
    # Visualize if requested
    if self.visualize:
        pallet_region_pcd_vis = o3d.geometry.PointCloud(pallet_region_pcd)
        pallet_region_pcd_vis.paint_uniform_color([0, 1, 0])  # Green
        o3d.visualization.draw_geometries(
            [pallet_region_pcd_vis],
            window_name="Isolated Pallet Region",
            width=800,
            height=600
        )
    
    return True

def debug_segmentation(self):
    """Visualize segmentation with height histogram for debugging"""
    import matplotlib.pyplot as plt
    
    # Get all points
    points = np.asarray(self.processed_pcd.points)
    heights = points[:, 2]
    
    # Create histogram
    plt.figure(figsize=(10, 6))
    hist, bins, _ = plt.hist(heights, bins=50, alpha=0.7)
    
    # Mark detected table and pallet heights if available
    if hasattr(self, 'table_height'):
        plt.axvline(self.table_height, color='gray', linestyle='--', linewidth=2, label='Table Height')
    
    if hasattr(self, 'pallet_height'):
        plt.axvline(self.pallet_height, color='cyan', linestyle='--', linewidth=2, label='Pallet Height')
    
    plt.title('Height Distribution in Point Cloud')
    plt.xlabel('Height (Z coordinate)')
    plt.ylabel('Number of Points')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Save the plot
    plt.savefig("height_histogram.png")
    print("Saved height histogram to 'height_histogram.png'")
    
    # Display if running in interactive environment
    try:
        plt.show()
    except:
        pass

def find_best_box_placement_on_table(table_pcd, box_dimensions, existing_boxes=[]):
    """Find the best placement for a box on the table"""
    # Extract points from the table surface
    points = np.asarray(table_pcd.points)
    
    # Calculate table boundaries
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    
    # Use the average Z height for placement
    table_height = np.mean(points[:, 2])
    
    # Create a grid of potential positions
    grid_size = 5  # Number of positions to try in each dimension
    x_range = np.linspace(min_bounds[0] + box_dimensions[0]/2, 
                          max_bounds[0] - box_dimensions[0]/2, grid_size)
    y_range = np.linspace(min_bounds[1] + box_dimensions[1]/2, 
                          max_bounds[1] - box_dimensions[1]/2, grid_size)
    
    best_score = -1
    best_position = None
    
    for x in x_range:
        for y in y_range:
            # Create a test box
            test_position = np.array([x, y, table_height + box_dimensions[2]/2])
            test_box = {
                'position': test_position,
                'dimensions': box_dimensions
            }
            
            # Skip if collision with existing boxes
            if check_collision(test_box, existing_boxes):
                continue
                
            # Calculate score based on distance from edges and other boxes
            edge_dist_x = min(x - min_bounds[0], max_bounds[0] - x) / box_dimensions[0]
            edge_dist_y = min(y - min_bounds[1], max_bounds[1] - y) / box_dimensions[1]
            edge_score = min(1.0, min(edge_dist_x, edge_dist_y))
            
            # Prefer positions away from other boxes but not too far
            box_distance_score = 1.0
            if existing_boxes:
                distances = []
                for box in existing_boxes:
                    ex_pos = box['position']
                    distance = np.sqrt((x - ex_pos[0])**2 + (y - ex_pos[1])**2)
                    distances.append(distance)
                
                min_dist = min(distances) if distances else 0
                box_distance_score = min(1.0, min_dist / (box_dimensions[0] + box_dimensions[1]))
            
            score = 0.7 * edge_score + 0.3 * box_distance_score
            
            if score > best_score:
                best_score = score
                best_position = test_position
    
    if best_position is None:
        return None
        
    return {
        'position': best_position,
        'dimensions': box_dimensions,
        'score': best_score,
        'placement_type': 'on_table'
    }