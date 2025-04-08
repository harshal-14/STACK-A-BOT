import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib import cm
import os
import sys
import time
import argparse


from .point_cloud_processor import (
    load_point_cloud,
    preprocess_point_cloud,
    segment_horizontal_surfaces,
    cluster_horizontal_surfaces
)

from .stability_utilities import (
    analyze_surface_flatness,
    calculate_surface_area,
    calculate_center_of_support,
    distance_from_edge,
    create_box_marker
)

def main(ply_file_path, box_size=(0.2, 0.15, 0.1), visualize=True, fix_orientation=True):
    """
    Main function to generate a stability map from a point cloud.
    
    Args:
        ply_file_path (str): Path to the PLY file
        box_size (tuple): Dimensions of the box (width, depth, height)
        visualize (bool): Whether to visualize the results
        fix_orientation (bool): Whether to fix upside-down orientation
        
    Returns:
        tuple: Best placement coordinates (x, y, z)
        float: Stability score at that location
    """
    output_dir = "stability_output"
    os.makedirs(output_dir, exist_ok=True)

    pcd = load_point_cloud(ply_file_path, fix_orientation=fix_orientation)
    o3d.io.write_point_cloud(os.path.join(output_dir, "original_point_cloud.ply"), pcd)
    print(f"Saved original point cloud to '{output_dir}/original_point_cloud.ply'")

    if visualize:
        print("Visualizing point cloud... (close window to continue)")
        o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")

    processed_pcd = preprocess_point_cloud(pcd)
    result = segment_horizontal_surfaces(processed_pcd, angle_threshold=60)
    if isinstance(result, tuple):
        horizontal_pcd = result[0]  # Extract just the point cloud
        rotation_info = result[1] if len(result) > 1 else None
    else:
        horizontal_pcd = result  # Old version compatibility
        rotation_info = None

    surface_clusters = cluster_horizontal_surfaces(horizontal_pcd)
    
    if not surface_clusters:
        print("\n==== NO SUITABLE SURFACES FOUND ====")
        print("Potential solutions:")
        print("1. Try with a different orientation fix (--no_orientation_fix)")
        print("2. Check if the point cloud is correctly scaled")
        print("3. Try adjusting parameters like voxel_size or angle_threshold")
        print("4. Check the generated files in the 'stability_output' directory")
        return None, 0.0
    
    best_overall_coordinates = None
    best_overall_score = -1
    best_surface = None
    best_stability_grid = None
    best_coordinate_grid = None
    
    for i, surface in enumerate(surface_clusters):
        print(f"Analyzing surface {i+1}/{len(surface_clusters)}...")
        
        

        stability_grid, coordinate_grid = evaluate_box_placement(surface, box_size)
        
        best_coords, best_score = find_best_placement(stability_grid, coordinate_grid)
        
        if best_coords is not None:
            print(f"Best placement on surface {i+1}: {best_coords}, score: {best_score:.4f}")
            
            o3d.io.write_point_cloud(os.path.join(output_dir, f"surface_{i+1}.ply"), surface)
            
            # Update the overall best placement if this one is better
            if best_score > best_overall_score:
                best_overall_score = best_score
                best_overall_coordinates = best_coords
                best_surface = surface
                best_stability_grid = stability_grid
                best_coordinate_grid = coordinate_grid
    
    if best_overall_coordinates is None:
        print("Could not find a suitable placement for the box.")
        return None, 0.0
    
    print(f"\n==== FINAL RESULT ====")
    print(f"Best overall placement: {best_overall_coordinates}")
    print(f"Stability score: {best_overall_score:.4f}")
    
    if visualize and best_surface is not None:

        o3d.visualization.draw_geometries([best_surface], window_name="Best Surface")
        
        # Re-evaluate the best surface
        visualize_stability_map(pcd, best_stability_grid, best_coordinate_grid, best_overall_coordinates)

    box_marker = create_box_marker(best_overall_coordinates, box_size)
    
    # Combine point cloud and box for visualization
    combined_pcd = o3d.geometry.PointCloud()
    combined_pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points))
    if pcd.has_colors():
        combined_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))
    else:
        combined_pcd.colors = o3d.utility.Vector3dVector(np.ones((len(pcd.points), 3)) * 0.7)
    
    # Visualize the combined result
    if visualize:
        o3d.visualization.draw_geometries([combined_pcd, box_marker], 
                                         window_name="Optimal Box Placement")

    # o3d.io.write_line_set(os.path.join(output_dir, "box_placement.ply"), box_marker)
    # print(f"Saved final placement visualization to '{output_dir}/box_placement.ply'")
    
    return best_overall_coordinates, best_overall_score

def evaluate_box_placement(surface_pcd, box_size=(0.2, 0.15, 0.1), existing_objects=None, 
                          proximity_weight=0.1, min_clearance=0.05):
    """
    Evaluate potential box placements on a surface.
    
    Args:
        surface_pcd (o3d.geometry.PointCloud): Point cloud of the surface
        box_size (tuple): Dimensions of the box (width, depth, height)
        
    Returns:
        numpy.ndarray: Grid of stability scores
        numpy.ndarray: Grid of 3D coordinates for each cell
    """
    # Get the BBx of the surface
    surface_bbox = surface_pcd.get_axis_aligned_bounding_box()
    min_bound = surface_bbox.min_bound
    max_bound = surface_bbox.max_bound
    
    # Define the grid resolution based on box size and point density
    points = np.asarray(surface_pcd.points)
    volume = np.prod(max_bound - min_bound)
    point_density = len(points) / volume
    
    # Adjust grid resolution based on point density
    grid_resolution = max(0.01, min(0.05, 1.0 / (point_density ** (1/3))))
    print(f"Using grid resolution of {grid_resolution:.4f} for stability map")
    
    # Ensure we have at least a few grid cells
    x_cells = max(5, int((max_bound[0] - min_bound[0]) / grid_resolution))
    y_cells = max(5, int((max_bound[1] - min_bound[1]) / grid_resolution))
    
    x_range = np.linspace(min_bound[0], max_bound[0], x_cells)
    y_range = np.linspace(min_bound[1], max_bound[1], y_cells)
    
    print(f"Created a {x_cells}×{y_cells} grid for stability evaluation")
    
    ## Algoithm goes like this:
    # 1. For each grid cell, calculate the stability score based on the following metrics:
    # Calculate average z height,
    # 2. Calculate support area percentage,
    # 3. Calculate surface flatness (use the flatness of the entire surface),
    # 4. Calculate support centering (distance from the center of the box),
    # 5. Calculate distance from edges of the box.
    # 6. Combine the scores (weighted).
    # 7. Return the stability grid and coordinates.
    # 8. Find the best placement based on the stability grid.
    # 9. Visualize the results. 
    avg_z = np.mean(points[:, 2])
    
    # Create grids to store scores and coordinates
    stability_grid = np.zeros((len(x_range), len(y_range)))
    coordinate_grid = np.zeros((len(x_range), len(y_range), 3))
    
    # Calculate metrics for the whole surface once
    total_area = calculate_surface_area(points)
    flatness = analyze_surface_flatness(points)
    
    print(f"Surface metrics - Total area: {total_area:.4f}, Flatness: {flatness:.4f}")
    
    for i, x in enumerate(x_range):
        for j, y in enumerate(y_range):
            # Define the Boxx FP on the surface
            box_min_x = x - box_size[0]/2
            box_max_x = x + box_size[0]/2
            box_min_y = y - box_size[1]/2
            box_max_y = y + box_size[1]/2
            
            # Filter points that fall within the  Boxx FP
            points_array = np.asarray(surface_pcd.points)
            mask = (
                (points_array[:, 0] >= box_min_x) &
                (points_array[:, 0] <= box_max_x) &
                (points_array[:, 1] >= box_min_y) &
                (points_array[:, 1] <= box_max_y)
            )
            box_support_points = points_array[mask]
            
            if len(box_support_points) == 0:
                stability_grid[i, j] = 0
                coordinate_grid[i, j] = [x, y, avg_z]
                continue
            
            # Create pcd for the support points
            support_pcd = o3d.geometry.PointCloud()
            support_pcd.points = o3d.utility.Vector3dVector(box_support_points)
            
            support_area = calculate_surface_area(box_support_points)
            box_footprint_area = box_size[0] * box_size[1]
            area_coverage = min(support_area / box_footprint_area, 1.0)

            box_center = np.array([x, y, avg_z])
            support_center = calculate_center_of_support(box_support_points)
            centering_error = np.linalg.norm(box_center[:2] - support_center[:2])
            centering_score = max(0, 1 - (centering_error / (box_size[0] / 2)))
            
            edge_distance = distance_from_edge(points, box_center)
            edge_score = min(edge_distance / (box_size[0] / 2), 1.0)
            
            proximity_score = 0.0
            if existing_objects is not None and len(existing_objects) > 0:
                # Calculate distance to nearest existing object
                nearest_distance = float('inf')
                for obj_center in existing_objects:
                    distance = np.linalg.norm(box_center[:2] - obj_center[:2])
                    # Subtract box size to measure from edge to edge
                    edge_distance = distance - (box_size[0]/2 + box_size[1]/2)/2  # approximate radius
                    # We want to be close but not too close
                    if edge_distance < min_clearance:
                        proximity_score = 0.0  # Too close
                    else:
                        # Higher score for closer placements (up to a limit)
                        proximity_score = max(0, 1 - (edge_distance / (box_size[0] * 2)))
                    nearest_distance = min(nearest_distance, distance)

            # Combine the scores (weighted)
            stability_score = (
                0.4 * area_coverage +  # 40% weight on support area
                0.3 * flatness +       # 30% weight on flatness
                0.2 * centering_score + # 20% weight on centering
                0.1 * edge_score        # 10% weight on edge distance
            )
            if existing_objects is not None:
                stability_score = (1 - proximity_weight) * stability_score + proximity_weight * proximity_score

            stability_grid[i, j] = stability_score
            coordinate_grid[i, j] = [x, y, avg_z]
    
    # Check if we found any stable positions
    if np.max(stability_grid) <= 0:
        print("WARNING: No stable positions found on this surface!")
    else:
        print(f"Maximum stability score: {np.max(stability_grid):.4f}")
    
    return stability_grid, coordinate_grid

def find_best_placement(stability_grid, coordinate_grid):
    """
    Find the best placement based on the stability grid.
    
    Args:
        stability_grid (numpy.ndarray): Grid of stability scores
        coordinate_grid (numpy.ndarray): Grid of 3D coordinates
        
    Returns:
        tuple: Best placement coordinates (x, y, z)
        float: Stability score at that location
    """
    # Check if stability grid is empty
    if stability_grid.size == 0:
        print("Empty stability grid, cannot find best placement.")
        return None, 0.0
        
    # Find the index of the maximum score
    max_index = np.unravel_index(np.argmax(stability_grid), stability_grid.shape)
    
    # Get the coordinates and score
    best_coordinates = coordinate_grid[max_index]
    best_score = stability_grid[max_index]
    
    print(f"Best placement found at coordinates: {best_coordinates}")
    print(f"Stability score: {best_score:.4f}")
    
    return best_coordinates, best_score

# TODO: Fix the visualization to show the box placement on the surface, currently it shows the box in the air
def visualize_stability_map(pcd, stability_grid, coordinate_grid, best_placement=None, rotation_info=None):
    """
    Visualize the stability map with counter-rotation to display in original orientation.
    
    Args:
        pcd (o3d.geometry.PointCloud): Original point cloud
        stability_grid (numpy.ndarray): Grid of stability scores
        coordinate_grid (numpy.ndarray): Grid of 3D coordinates
        best_placement (tuple, optional): Best placement coordinates
        rotation_info (dict, optional): Information about rotations applied during processing
        
    Returns:
        None
    """
    fig = plt.figure(figsize=(18, 12))
    
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Get points from the original point cloud
    points = np.asarray(pcd.points)
    
    # Create a counter-rotation transformation to reverse any rotations applied
    if rotation_info is not None and rotation_info.get('applied_rotation', False):
        # Use the inverse of the stored rotation matrix
        rotation_matrix = np.array(rotation_info['rotation_matrix'])
        # Compute the inverse (transpose for rotation matrices)
        counter_rotation = rotation_matrix.T
        print("Using stored rotation information to counter-rotate visualization")
    else:
        # Default counter-rotation if no rotation info available
        # This is the inverse of the standard rotation matrix used in load_point_cloud()
        # [1, 0, 0; 0, -1, 0; 0, 0, -1] which is its own inverse
        counter_rotation = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        print("Using default counter-rotation for visualization")
    
    # Apply counter-rotation to visualization points (not to the actual point cloud)
    vis_points = np.copy(points)
    for i in range(len(vis_points)):
        vis_points[i] = counter_rotation @ vis_points[i]
    
    # Plot the counter-rotated points in gray
    ax1.scatter(vis_points[:, 0], vis_points[:, 1], vis_points[:, 2], 
                c='gray', s=1, alpha=0.3)
    
    # Handle stability grid points
    x_coords = coordinate_grid[:, :, 0].flatten()
    y_coords = coordinate_grid[:, :, 1].flatten()
    z_coords = coordinate_grid[:, :, 2].flatten()
    scores = stability_grid.flatten()
    
    # Only plot points with non-zero scores
    mask = scores > 0
    
    # Counter-rotate stability grid points as well
    stability_points = np.vstack([x_coords[mask], y_coords[mask], z_coords[mask]]).T
    rotated_stability_points = np.zeros_like(stability_points)
    
    for i in range(len(stability_points)):
        rotated_stability_points[i] = counter_rotation @ stability_points[i]
    
    # Plot counter-rotated stability points with their scores
    scatter = ax1.scatter(rotated_stability_points[:, 0], 
                         rotated_stability_points[:, 1], 
                         rotated_stability_points[:, 2],
                         c=scores[mask], cmap='viridis', 
                         s=30, alpha=0.7, edgecolors='none')
    
    cbar = plt.colorbar(scatter, ax=ax1)
    cbar.set_label('Stability Score')
    
    # Highlight the best placement if provided
    if best_placement is not None:
        # Counter-rotate the best placement point
        rotated_best = counter_rotation @ best_placement
        ax1.scatter(rotated_best[0], rotated_best[1], rotated_best[2],
                    c='red', s=100, marker='*', edgecolors='black', linewidth=1.5)
    
    # Set labels and title
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Stability Map for Box Placement (Original Orientation)')
    
    # Adjust the view angle to show horizontal surfaces properly
    # This typically gives a clearer view of horizontal planes
    ax1.view_init(elev=30, azim=45)
    
    plt.tight_layout()
    plt.savefig('stability_map.png', dpi=300, bbox_inches='tight')
    print("Saved visualization to 'stability_map.png'")
    
    # Also save the 3D point cloud with stability information for viewing in external tools
    vis_pcd = o3d.geometry.PointCloud()
    vis_pcd.points = o3d.utility.Vector3dVector(vis_points)  # Use the counter-rotated points
    
    colors = np.ones((len(vis_points), 3)) * 0.7  # Default gray
    
    if mask.any():
        stable_colors = cm.viridis(scores[mask]/max(scores[mask]))[:, :3]
        
        # Write a separate colored point cloud with just the stability map
        stable_pcd = o3d.geometry.PointCloud()
        stable_pcd.points = o3d.utility.Vector3dVector(rotated_stability_points)  # Use counter-rotated points
        stable_pcd.colors = o3d.utility.Vector3dVector(stable_colors)
        o3d.io.write_point_cloud("stability_map.ply", stable_pcd)
        print("Saved colored point cloud to 'stability_map.ply'")
    
    plt.show()
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a stability map from a point cloud for box placement")
    parser.add_argument("ply_file", help="Path to the PLY file")
    parser.add_argument("--box_width", type=float, default=0.2, help="Width of the box in meters")
    parser.add_argument("--box_depth", type=float, default=0.15, help="Depth of the box in meters")
    parser.add_argument("--box_height", type=float, default=0.1, help="Height of the box in meters")
    parser.add_argument("--no_viz", action="store_true", help="Disable visualization")
    parser.add_argument("--no_orientation_fix", action="store_true", help="Disable automatic orientation fix")
    parser.add_argument("--voxel_size", type=float, default=0.01, help="Voxel size for downsampling (smaller = more detail)")
    parser.add_argument("--angle_threshold", type=float, default=45, help="Angle threshold in degrees for horizontal surface detection")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode with additional visualizations")
    
    args = parser.parse_args()
    
    if args.debug:
        print("Debug mode enabled - additional visualizations will be generated")
    
    best_placement, score = main(
        args.ply_file,
        box_size=(args.box_width, args.box_depth, args.box_height),
        visualize=not args.no_viz,
        fix_orientation=not args.no_orientation_fix
    )