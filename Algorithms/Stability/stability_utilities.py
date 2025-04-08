import numpy as np
import cv2
import os
import sys
from scipy.spatial import ConvexHull
from sklearn.decomposition import PCA
import open3d as o3d

def analyze_surface_flatness(points):
    """
    Analyze the flatness of a surface using PCA/

    Args:
        points (numpy.ndarray): 3D points representing the surface.

    Returns:
        float" Flatness score (0-1, higher is flatter).
    """
    pca = PCA(n_components=3)
    pca.fit(points)

    eigenvalues = pca.explained_variance_ # smallest eigenvalue indicates the direction of the variance perpendicular to the surface
    flatness = 1.0 - (eigenvalues[2] / np.sum(eigenvalues)) # Higher is flatter
    print(f"Flatness score: {flatness}")

    return flatness

def calculate_surface_area(points):
    """
    Calculate the approx. surface area using 2D convex hull.
    
    Args:
        points (numpy.ndarray) : Points representing the surface.
    
    Returns:
        float: Approx. surface area.
    """
    xy_points = points[:, :2] # Project the points to 2D XY plane(assuming Z is up)

    try:
        hull = ConvexHull(xy_points) # Compute the convex hull of the points
        return hull.volume # The volume of the convex hull is the area in 2D
    except Exception as e:
        print(f"Error calculating surface area: {e}")
        return 0.0

def calculate_center_of_support(points):
    """
    Calculate the center of support for the given points to a surface.

    Args:
        points (numpy.ndarray): Points representing the surface.

    Returns:
        numpy.ndarray: Center of support (x, y, z).
    """
    center = np.mean(points, axis=0) # Calculate the mean of the points
    return center

def distance_from_edge(points, point):
    """
    Calculate the minimum distance from a point to the edge of the surface.

    Args:
        points (numpy.ndarray) : Points representing the surface.
        point (numpy.ndarray) : Point to measure distance from.

    Returns:
        float: Minimum distance from point to edge of surface.
    """
    xy_points = points[:, :2] # Project the points to 2D XY plane(assuming Z is up)
    xy_point = point[:2] 
    
    try:
        hull = ConvexHull(xy_points) # calculate the convex hull in 2D
        boundary_poitns = xy_points[hull.vertices] # take the points for the boundary of the hull

        # try calculating the distance from the pt to line segments of the hull boundary
        min_dist = float('inf')

        for i in range(len(boundary_poitns)):
            p1 = boundary_poitns[i]
            p2 = boundary_poitns[(i + 1) % len(boundary_poitns)]
            
            # calculate the distance
            line_vec = p2 - p1
            point_vec = xy_point - p1
            line_len = np.linalg.norm(line_vec)
            line_unit_vec = line_vec / line_len if line_len > 0 else np.zeros_like(line_vec)

            point_projection = np.dot(point_vec, line_unit_vec)

            # conditions checking the distance bsaed on the projection
            if point_projection < 0:
                dist = np.linalg.norm(xy_point - p1) # close to p1 projections outside line segment
            elif point_projection > line_len:
                dist = np.linalg.norm(xy_point - p2) # close to p2 projections outside line segment
            else:
                point_projection = p1 + line_unit_vec * point_projection # projection of the point on the line segment
                dist = np.linalg.norm(xy_point - point_projection)

            min_dist = min(min_dist, dist)

        return min_dist # return the minimum distance from the point to the edge of the surface
    except Exception as e:
        print(f"Error calculating distance from edge: {e}")
        return 0.0

def create_box_marker(position, box_size, surface_normal=None):
    """
    Create a box marker at the specified position, aligned with the surface.
    
    Args:
        position (numpy.ndarray): Center position of the bottom face of the box (x, y, z)
        box_size (tuple): Dimensions of the box (width, depth, height)
        surface_normal (numpy.ndarray, optional): Normal vector of the surface
        
    Returns:
        o3d.geometry.LineSet: Line set representing the box
    """
    import open3d as o3d
    
    # Default normal pointing upward if not provided
    if surface_normal is None:
        surface_normal = np.array([0, 0, 1])
    
    # Normalize the surface normal
    surface_normal = surface_normal / np.linalg.norm(surface_normal)
    
    # Calculate the center of the box (shift up by half height from position)
    # This ensures the bottom of the box is at the specified position
    box_center = position + surface_normal * (box_size[2] / 2)
    
    half_width = box_size[0] / 2
    half_depth = box_size[1] / 2
    half_height = box_size[2] / 2
    
    # If the surface is not horizontal, we need to align the box with the surface
    if not np.allclose(surface_normal, np.array([0, 0, 1])):
        # Calculate rotation matrix to align [0,0,1] with surface_normal
        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(z_axis, surface_normal)
        
        # If rotation axis is too small, surfaces are already aligned
        if np.linalg.norm(rotation_axis) > 1e-6:
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            angle = np.arccos(np.dot(z_axis, surface_normal))
            
            # Rodrigues rotation formula to create rotation matrix
            K = np.array([
                [0, -rotation_axis[2], rotation_axis[1]],
                [rotation_axis[2], 0, -rotation_axis[0]],
                [-rotation_axis[1], rotation_axis[0], 0]
            ])
            
            rotation_matrix = (
                np.eye(3) + 
                np.sin(angle) * K + 
                (1 - np.cos(angle)) * (K @ K)
            )
        else:
            rotation_matrix = np.eye(3)
            
        # Create vertices for a standard box
        vertices = np.array([
            [-half_width, -half_depth, -half_height],  # 0: bottom face, left back
            [half_width, -half_depth, -half_height],   # 1: bottom face, right back
            [half_width, half_depth, -half_height],    # 2: bottom face, right front
            [-half_width, half_depth, -half_height],   # 3: bottom face, left front
            [-half_width, -half_depth, half_height],   # 4: top face, left back
            [half_width, -half_depth, half_height],    # 5: top face, right back
            [half_width, half_depth, half_height],     # 6: top face, right front
            [-half_width, half_depth, half_height]     # 7: top face, left front
        ])
        
        # Rotate and translate the vertices
        rotated_vertices = []
        for vertex in vertices:
            # Rotate the vertex
            rotated_vertex = rotation_matrix @ vertex
            # Translate to the final position
            rotated_vertex = rotated_vertex + box_center
            rotated_vertices.append(rotated_vertex)
        
        vertices = rotated_vertices
    else:
        # Standard box vertices if no rotation needed
        vertices = [
            [box_center[0] - half_width, box_center[1] - half_depth, box_center[2] - half_height],  # 0
            [box_center[0] + half_width, box_center[1] - half_depth, box_center[2] - half_height],  # 1
            [box_center[0] + half_width, box_center[1] + half_depth, box_center[2] - half_height],  # 2
            [box_center[0] - half_width, box_center[1] + half_depth, box_center[2] - half_height],  # 3
            [box_center[0] - half_width, box_center[1] - half_depth, box_center[2] + half_height],  # 4
            [box_center[0] + half_width, box_center[1] - half_depth, box_center[2] + half_height],  # 5
            [box_center[0] + half_width, box_center[1] + half_depth, box_center[2] + half_height],  # 6
            [box_center[0] - half_width, box_center[1] + half_depth, box_center[2] + half_height]   # 7
        ]
    
    # 12 lines connecting the vertices
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
        [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
        [0, 4], [1, 5], [2, 6], [3, 7]   # Connecting edges
    ]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    colors = [[1, 0, 0] for _ in range(len(lines))]
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set