import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib import cm
import os
import sys
import time
import argparse

from point_cloud_processor import (
    load_point_cloud,
    preprocess_point_cloud,
    segment_horizontal_surfaces,
    cluster_horizontal_surfaces
)

from stability_utilities import (
    analyze_surface_flatness,
    calculate_surface_area,
    calculate_center_of_support,
    distance_from_edge,
    create_box_marker
)

#TODO: Finish writing them main function for stability generator
def evaluate_box_placement(surface_pcd, box_size=(0.2, 0.15, 0.1)):
    pass

def find_best_placement(stability_grid, coordinate_grid):
    pass

def visualize_stability_map(pcd, stability_grid, coordinate_grid, best_placement = None):
    pass


    