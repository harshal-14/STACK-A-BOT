import numpy as np
import open3d as o3d
import os


#TODO: Implement the functions, to make stability generator work
def load_point_cloud(file_path, fix_orientation = True):
    pass

def preprocess_point_cloud(pcd, voxel_size = 0.01):
    pass

def segment_horizontal_surfaces(pcd, angle_threshold = 45):
    pass

def segment_planes_as_horizontal(pcd, distance_threshold = 0.01, ransac_n = 3, num_iterations = 1000):
    pass

def cluster_horizontal_surfaces(pcd, eps = 0.05, min_points = 20):
    pass