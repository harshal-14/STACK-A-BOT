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

def create_box_marker(position, box_size):
    """
    Create a box marker at the specified position.
    
    Args:
        position (numpy.ndarray): Center position of the box (x, y, z)
        box_size (tuple): Dimensions of the box (width, depth, height)
        
    Returns:
        o3d.geometry.LineSet: Line set representing the box
    """
    import open3d as o3d
    
    half_width = box_size[0] / 2
    half_depth = box_size[1] / 2
    half_height = box_size[2] / 2
    #TODO: Check the order of the vertices and the lines
    vertices = [
        [position[0] - half_width, position[1] - half_depth, position[2] - half_height],  # 0
        [position[0] + half_width, position[1] - half_depth, position[2] - half_height],  # 1
        [position[0] + half_width, position[1] + half_depth, position[2] - half_height],  # 2
        [position[0] - half_width, position[1] + half_depth, position[2] - half_height],  # 3
        [position[0] - half_width, position[1] - half_depth, position[2] + half_height],  # 4
        [position[0] + half_width, position[1] - half_depth, position[2] + half_height],  # 5
        [position[0] + half_width, position[1] + half_depth, position[2] + half_height],  # 6
        [position[0] - half_width, position[1] + half_depth, position[2] + half_height]   # 7, am I forgetting something??
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