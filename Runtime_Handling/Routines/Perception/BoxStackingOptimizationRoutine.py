"""
Routine for optimizing box stacking using 3D point cloud data.
"""

import numpy as np
import os
import open3d as o3d

from ..Routine import Routine
from ...Status import Status, Condition
from ....Algorithms.Stability.point_cloud_processor import (
    load_point_cloud,
    preprocess_point_cloud,
    align_point_cloud
)
from ....Algorithms.Stability.box_stacking_utils import (
    create_box_marker,
    detect_existing_boxes_from_point_cloud,
    apply_direct_scaling,
    place_boxes_on_pallet,
    debug_segmentation,
    segment_pallet_from_table,
    isolate_pallet_for_stacking,
    visualize_box_placements,
)

class BoxStackingOptimizationRoutine(Routine):
    """Routine for optimizing box stacking using 3D point cloud data.
    
    This routine processes a 3D point cloud to identify boxes and table surfaces,
    then determines optimal stacking configurations based on box dimensions.
    
    Attributes:
        ply_file_path (str): Path to the PLY file containing the point cloud
        box_sizes (list): List of box dimensions (width, depth, height) in meters
        visualize (bool): Whether to generate visualizations during processing
        fix_orientation (bool): Whether to automatically fix point cloud orientation
        placed_boxes (list): Information about the placed boxes
    """

    def __init__(self, ply_file_path, box_sizes=None, visualize=True, fix_orientation=True, use_simple_method=False, known_pallet_dims=None):
        """Initialize the box stacking optimization routine.
        
        Args:
            ply_file_path (str): Path to the PLY file
            box_sizes (list): List of box dimensions (width, depth, height) in meters
            visualize (bool): Whether to generate visualizations
            fix_orientation (bool): Whether to apply orientation correction
        """
        self.ply_file_path = ply_file_path
        self.box_sizes = box_sizes or [(0.2, 0.15, 0.1)]  # Default box size if none provided
        self.visualize = visualize
        self.fix_orientation = fix_orientation
        self.placed_boxes = []
        self.output_dir = "box_stacking_output"
        
        # Internal state variables
        self.raw_pcd = None
        self.processed_pcd = None
        self.table_pcd = None
        self.objects_pcd = None
        self.detected_boxes = []
        self.box_surfaces = []
        self.layers = []  
        self.current_layer_index = 0
        self.layer_height_tolerance = 0.01 
        self.grid_resolution = 0.005
        self.scaling_applied = True
        self.use_simple_method = use_simple_method
        self.known_pallet_dims = known_pallet_dims  # Known pallet dimensions (if provided)

    # Bridge methods for RoutineScheduler
    def _init(self, prev_outputs):
        return self.init(prev_outputs)
        
    def _loop(self):
        return self.loop()
        
    def _end(self):
        return self.end()
        
    def _handle_fault(self, prev_status=None):
        # Create a default Status object if prev_status is None
        if prev_status is None:
            prev_status = Status(
                Condition.Fault,
                err_msg="Unknown fault in box stacking routine",
                err_type=RuntimeError
            )
        return self.handle_fault(prev_status)
    
    def find_optimal_box_placement(self, box_size):
        """Find best box placement optimizing for volumetric efficiency"""
        # Ensure we have initialized the first layer
        if not self.layers:
            if not self.initialize_first_layer():
                return None
        
        # Try to place in current layer first
        layer_placement = self._find_layer_placement(box_size)
        
        if layer_placement:
            print(f"Found placement in layer {self.current_layer_index}")
            return layer_placement
        
        # If current layer is full, try to create a new layer
        print(f"Layer {self.current_layer_index} is full or unsuitable")
        if self.create_next_layer():
            return self._find_layer_placement(box_size)
        
        print("Could not find placement in any layer")
        return None

    def _find_layer_placement(self, box_size):
        """Find optimal position within the current layer"""
        if self.current_layer_index >= len(self.layers):
            return None
            
        layer = self.layers[self.current_layer_index]
        grid_positions = self.get_layer_grid()
        
        if not grid_positions:
            return None
        
        if len(layer['boxes']) >=3:
            print(f"Layer {self.current_layer_index} is full")
            return None

        # Evaluate each grid position
        placement_options = []
        
        for grid_item in grid_positions:
            position = grid_item['position'].copy()
            grid_pos = grid_item['grid_pos']

            # Create a test box
            test_box = {
                'position': position,
                'dimensions': box_size
            }
            
            # Perform a detailed collision check
            collision = self.check_collision(test_box, self.placed_boxes, tolerance=0.002)
            
            # Skip this position if collision detected
            if collision:
                continue
            
            # If debugging is enabled, visualize first few collision checks
            # if self.visualize and len(placement_options) < 3:
            #     self.visualize_collision_check(test_box, self.placed_boxes, collision)
            
            # Adjust Z position based on layer height
            position[2] += box_size[2] / 2
            
            # Calculate how many grid cells this box would occupy
            cells_x = max(1, int(box_size[0] / self.grid_resolution))
            cells_y = max(1, int(box_size[1] / self.grid_resolution))
            
            # Check if all required cells are available
            can_place = True
            occupied_cells = set()
            
            for dx in range(cells_x):
                for dy in range(cells_y):
                    test_pos = (grid_pos[0] + dx, grid_pos[1] + dy)
                    if test_pos in layer['occupied_cells']:
                        can_place = False
                        break
                    occupied_cells.add(test_pos)
                
                if not can_place:
                    break
            
            if not can_place:
                continue
            
            # Check for collisions with existing boxes
            test_box = {
                'position': position,
                'dimensions': box_size
            }
            
            if self.check_collision(test_box, self.placed_boxes):
                continue
            
            # Calculate stability score
            stability_score = 1.0  # Default high for first layer
            
            if self.current_layer_index > 0:
                # For higher layers, check support from layer below
                prev_layer = self.layers[self.current_layer_index - 1]
                support_score = self._calculate_support_score(test_box, prev_layer['boxes'])
                if support_score < 0.7:  # Minimum support threshold
                    continue
                stability_score = support_score
            
            # Calculate compactness score (how well it fits with other boxes)
            compactness_score = self._calculate_compactness_score(test_box, layer['boxes'])

            centrality_score = self._calculate_centrality_score(position)

            box_proximity = self._calculate_box_to_box_proximity(position, box_size)
            object_proximity = self._calculate_proximity_to_objects(position)
        
            # Weighted final score
            if len(self.placed_boxes) == 0:
                # First box should be near detected objects
                final_score = 0.8 * object_proximity + 0.2 * centrality_score
            else:
                # Subsequent boxes should prioritize packing density
                final_score = 0.6 * box_proximity + 0.3 * object_proximity + 0.1 * centrality_score
            
            placement_options.append({
                'position': position,
                'dimensions': box_size,
                'score': final_score,
                'centrality': centrality_score,  # Store for debugging
                'placement_type': 'layer_based',
                'layer_index': self.current_layer_index,
                'occupied_cells': occupied_cells
            })
        
        if not placement_options:
            return None
        
        # Select the best option
        best_placement = max(placement_options, key=lambda x: x['score'])
        
        # Mark cells as occupied
        layer['occupied_cells'].update(best_placement['occupied_cells'])
        
        # Add to layer's boxes
        layer['boxes'].append(best_placement)
        
        return best_placement

    def _calculate_support_score(self, box, supporting_boxes):
        """Calculate how well a box is supported by boxes in the layer below, if any"""
        if not supporting_boxes:
            return 0.0
        
        box_min_x = box['position'][0] - box['dimensions'][0]/2
        box_max_x = box['position'][0] + box['dimensions'][0]/2
        box_min_y = box['position'][1] - box['dimensions'][1]/2
        box_max_y = box['position'][1] + box['dimensions'][1]/2
        
        # Calculate footprint area
        box_area = box['dimensions'][0] * box['dimensions'][1]
        
        # Calculate supported area
        supported_area = 0
        
        for support_box in supporting_boxes:
            sup_min_x = support_box['position'][0] - support_box['dimensions'][0]/2
            sup_max_x = support_box['position'][0] + support_box['dimensions'][0]/2
            sup_min_y = support_box['position'][1] - support_box['dimensions'][1]/2
            sup_max_y = support_box['position'][1] + support_box['dimensions'][1]/2
            
            # Calculate overlap area
            overlap_width = max(0, min(box_max_x, sup_max_x) - max(box_min_x, sup_min_x))
            overlap_height = max(0, min(box_max_y, sup_max_y) - max(box_min_y, sup_min_y))
            overlap_area = overlap_width * overlap_height
            
            supported_area += overlap_area
        
        # Support ratio
        support_ratio = min(1.0, supported_area / box_area)
        
        # Apply threshold - need minimum 70% support
        if support_ratio < 0.7:
            return 0.0
        
        return support_ratio

    def _calculate_compactness_score(self, box, existing_boxes):
        """Calculate how compactly this box fits with existing boxes"""
        if not existing_boxes:
            return 1.0  # First box in layer is good
        
        # Calculate average distance to other boxes (closer is better)
        min_distances = []
        
        for other_box in existing_boxes:
            # Distance between box centers in XY plane
            distance = np.sqrt(
                (box['position'][0] - other_box['position'][0])**2 +
                (box['position'][1] - other_box['position'][1])**2
            )
            
            # Minimum possible distance for touching boxes
            min_possible = (box['dimensions'][0] + other_box['dimensions'][0])/2 + \
                        (box['dimensions'][1] + other_box['dimensions'][1])/2
            
            # Normalized distance (1.0 = touching, <1.0 = overlapping, >1.0 = gap)
            norm_distance = distance / min_possible
            
            min_distances.append(norm_distance)
        
        # Take minimum distance to any box
        if min_distances:
            min_dist = min(min_distances)
            # Ideal is boxes touching (norm_distance = 1.0)
            # Score decreases as we move away from 1.0
            return max(0.0, 1.0 - abs(min_dist - 1.0))
        
        return 0.5  # Default

    def print_scene_details(self, pcd, table_pcd=None, objects_pcd=None, box_sizes=None):
        """Print detailed information about the scene for debugging"""
        print("\n=== SCENE DETAILS ===")
        
        # Overall point cloud information
        bbox = pcd.get_axis_aligned_bounding_box()
        extent = bbox.get_extent()
        center = bbox.get_center()
        
        print(f"\n=== POINT CLOUD DETAILS ===")
        print(f"Point cloud has {len(pcd.points)} points")
        print(f"Bounding box dimensions (width, depth, height): {extent}")
        print(f"Bounding box center: {center}")
        print(f"Max dimension: {max(extent)}")
        print(f"Min dimension: {min(extent)}")
        
        # Calculate approximate scale
        if max(extent) < 0.1:
            print("WARNING: Point cloud appears to be in MILLIMETERS (max dimension < 0.1)")
        elif max(extent) < 10:
            print("Point cloud appears to be in METERS (max dimension between 0.1 and 10)")
        else:
            print("WARNING: Point cloud may be in CENTIMETERS or another unit (max dimension > 10)")
        
        # Table and object information
        if table_pcd is not None:
            table_bbox = table_pcd.get_axis_aligned_bounding_box()
            table_extent = table_bbox.get_extent()
            print(f"\nTable surface has {len(table_pcd.points)} points")
            print(f"Table dimensions: {table_extent}")
            print(f"Table height (Z) range: {table_bbox.min_bound[2]:.6f} to {table_bbox.max_bound[2]:.6f}")
        
        if objects_pcd is not None:
            print(f"\nObject point cloud has {len(objects_pcd.points)} points")
            if len(objects_pcd.points) > 0:
                obj_bbox = objects_pcd.get_axis_aligned_bounding_box()
                obj_extent = obj_bbox.get_extent()
                print(f"Object cloud dimensions: {obj_extent}")
                print(f"Object height (Z) range: {obj_bbox.min_bound[2]:.6f} to {obj_bbox.max_bound[2]:.6f}")
        
        # Compare box sizes to point cloud
        if box_sizes:
            print("\n=== BOX SIZE ANALYSIS ===")
            avg_box_size = np.mean([np.mean(size) for size in box_sizes])
            max_box_dim = max([max(size) for size in box_sizes])
            
            # Calculate ratio of box size to point cloud size
            size_ratio = max_box_dim / max(extent)
            
            print(f"Number of boxes to place: {len(box_sizes)}")
            print(f"Average box dimension: {avg_box_size:.6f}")
            print(f"Maximum box dimension: {max_box_dim:.6f}")
            print(f"Ratio of max box dimension to point cloud max dimension: {size_ratio:.6f}")
            
            if size_ratio > 0.5:
                print("WARNING: Boxes are very large compared to the point cloud!")
                print(f"Recommended box dimensions: scale down by factor of approximately {size_ratio * 2:.1f}")
            elif size_ratio < 0.001:
                print("WARNING: Boxes are very small compared to the point cloud!")
                print(f"Recommended box dimensions: scale up by factor of approximately {0.01 / size_ratio:.1f}")

    def save_robot_coordinates(self, robot_coordinates):
        """Save robot coordinates to a file for use with the robot"""
        filepath = os.path.join(self.output_dir, "robot_coordinates.txt")
        
        try:
            with open(filepath, "w") as f:
                f.write("# Box coordinates for robot placement\n")
                f.write("# Format: box_id, x, y, z, width, depth, height, placement_type\n\n")
                
                for box in robot_coordinates:
                    pos = box['position']
                    dims = box['dimensions']
                    f.write(f"{box['id']}, {pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f}, ")
                    f.write(f"{dims[0]:.6f}, {dims[1]:.6f}, {dims[2]:.6f}, ")
                    f.write(f"{box['placement_type']}\n")
                    
            print(f"Saved robot coordinates to {filepath}")
        except Exception as e:
            print(f"Error saving robot coordinates: {e}")

    def init(self, prev_outputs, parameters=None) -> Status:
        """Initialize the box stacking optimization.
        
        Args:
            prev_outputs (dict): Outputs from previous routines
            parameters (dict, optional): Additional parameters
            
        Returns:
            Status: Success status if initialization is successful
        """
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # We can use parameters from previous routines if needed
        if parameters is not None:
            if 'ply_file_path' in parameters:
                self.ply_file_path = parameters['ply_file_path']
            if 'box_sizes' in parameters:
                self.box_sizes = parameters['box_sizes']
            if 'output_dir' in parameters:
                self.output_dir = parameters['output_dir']
                
        # Validate input file existence
        if not os.path.exists(self.ply_file_path):
            return Status(
                Condition.Fault,
                err_msg=f"Point cloud file not found: {self.ply_file_path}",
                err_type=FileNotFoundError
            )
            
        print(f"Initialized box stacking optimization for point cloud: {self.ply_file_path}")
        print(f"Box dimensions to place: {self.box_sizes}")
        print(f"Output directory: {self.output_dir}")
            
        return Status(Condition.Success)

    def visualize_with_coordinate_frame(self, pcd, size=0.1):
        """Visualize point cloud with coordinate frame to show scale"""
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        o3d.visualization.draw_geometries([pcd, coordinate_frame], 
                                        window_name=f"Point Cloud with {size}m Coordinate Frame")

    def loop(self, use_simple_method=False) -> Status:
        """Process the point cloud and optimize box stacking."""
        try:
            # Step 1: Load and process the point cloud
            print("Loading point cloud...")
            self.raw_pcd = load_point_cloud(self.ply_file_path, self.fix_orientation)
            
            print("Preprocessing point cloud...")
            self.processed_pcd = preprocess_point_cloud(self.raw_pcd)
            
            # Step 2: Align point cloud with coordinate system
            print("Aligning point cloud...")
            self.processed_pcd = align_point_cloud(self.processed_pcd)
            
            # Step 3: Segment table, pallet, and objects
            print("Segmenting table, pallet, and objects...")
            self.table_pcd, self.pallet_pcd, self.objects_pcd = segment_pallet_from_table(self,
                self.processed_pcd, 
                visualize=self.visualize
            )
            
            # Print detailed scene information
            self.print_scene_details(
                self.processed_pcd, 
                self.pallet_pcd, 
                self.objects_pcd, 
                self.box_sizes
            )
            
            # Decide which placement method to use
            if use_simple_method:
                print("\n=== Using Simple Box Stacking Algorithm ===")
                self.placed_boxes = self.simple_box_placement()
                
                
                # Generate robot coordinates
                robot_coordinates = self.get_robot_coordinates()
                
                # Print robot coordinates for use with the robot
                print("\n=== ROBOT COORDINATES ===")
                for box in robot_coordinates:
                    print(f"Box {box['id']}: Position {box['position']}, Dimensions {box['dimensions']}")
                
                # Save robot coordinates to file
                self.save_robot_coordinates(robot_coordinates)
                
                # Visualize if requested
                if self.visualize and self.placed_boxes:
                    visualize_box_placements(self)
                    
            else:
                # Debug visualization of height distribution
                debug_segmentation(self)
                
                # Step 5: Focus on the pallet for stacking
                if hasattr(self, 'pallet_pcd') and len(self.pallet_pcd.points) > 50:
                    # Isolate just the pallet region for analysis
                    if isolate_pallet_for_stacking(self):
                        # Use the isolated pallet region for stacking
                        place_boxes_on_pallet(self)
                        self.stacking_surface = self.pallet_pcd
                        # Use the cropped point cloud for object detection
                        self.processed_pcd = self.pallet_region_pcd
                        print("Using isolated pallet region for box stacking")
                    else:
                        print("Failed to isolate pallet, using full pallet surface")
                        self.stacking_surface = self.pallet_pcd
                else:
                    print("No pallet detected, using table surface")
                    self.stacking_surface = self.table_pcd
                
                # Step 6: Detect existing objects/boxes in the scene
                detected_boxes = detect_existing_boxes_from_point_cloud(self)
                self.placed_boxes = detected_boxes.copy()  # Start with detected boxes
                
                print(f"Detected {len(detected_boxes)} existing boxes in the scene")
                
                # Step 7: Initialize layers
                self.layers = []
                if self.initialize_first_layer():
                    # If we have detected boxes, add them to layer 0
                    if detected_boxes:
                        self.layers[0]['boxes'].extend(detected_boxes)
                        print(f"Added {len(detected_boxes)} detected boxes to layer 0")
                        
                        # Visualize detected boxes if requested
                        if self.visualize:
                            self._visualize_current_stacking()
                
                # Step 8: Optimize box stacking for new boxes
                print("\n=== Optimizing Box Placement for Volumetric Efficiency ===")
                
                # Sort boxes by base area (largest footprint first)
                sorted_sizes = sorted(self.box_sizes, 
                                    key=lambda size: size[0] * size[1], 
                                    reverse=True)
                
                for i, box_size in enumerate(sorted_sizes):
                    print(f"\n==== PLACING BOX {i+1}/{len(sorted_sizes)} ====")
                    print(f"Box dimensions: {box_size}")
                    
                    # Find the best placement using layer-based approach
                    placement = self.find_optimal_box_placement(box_size)
                    
                    if placement:
                        # Double-check for collisions before finalizing
                        collision = self.check_collision(placement, self.placed_boxes)
                        
                        if collision:
                            print("WARNING: Collision detected after placement! Skipping this box.")
                            if self.visualize:
                                self.visualize_collision_check( placement, self.placed_boxes, True)
                            continue
                            
                        print(f"Placed box at position: {placement['position']}")
                        print(f"Layer: {placement.get('layer_index', 0)}")
                        print(f"Stability score: {placement.get('score', 0):.4f}")
                        
                        # Add to placed boxes list and current layer
                        self.placed_boxes.append(placement)
                        
                        # Ensure the box is added to the correct layer
                        layer_index = placement.get('layer_index', 0)
                        if layer_index < len(self.layers):
                            self.layers[layer_index]['boxes'].append(placement)
                        
                        # Visualize if requested
                        if self.visualize:
                            self._visualize_current_stacking()
                    else:
                        print(f"Could not find a suitable placement for box {i+1}")
                        
                        # Try creating a new layer if we can't place in current layer
                        if self.create_next_layer():
                            print(f"Created new layer {self.current_layer_index}, trying again...")
                            placement = self.find_optimal_box_placement( box_size)
                            
                            if placement:
                                # Add to placed boxes list and current layer
                                self.placed_boxes.append(placement)
                                
                                # Ensure the box is added to the correct layer
                                layer_index = placement.get('layer_index', 0)
                                if layer_index < len(self.layers):
                                    self.layers[layer_index]['boxes'].append(placement)
                                
                                # Visualize if requested
                                if self.visualize:
                                    self._visualize_current_stacking()
                            else:
                                print(f"Still could not place box {i+1}, skipping")
                        else:
                            print(f"Could not create a new layer, skipping box {i+1}")
                
                # Step 9: Calculate packing efficiency
                self.calculate_packing_efficiency()
                
                # Step 10: Generate robot coordinates
                robot_coordinates = self.get_robot_coordinates()
                
                # Print robot coordinates for use with the robot
                print("\n=== ROBOT COORDINATES ===")
                for box in robot_coordinates:
                    print(f"Box {box['id']}: Position {box['position']}, Dimensions {box['dimensions']}")
                
                # Save robot coordinates to file
                self.save_robot_coordinates( robot_coordinates)
                
                # Final visualization
                if self.visualize and self.placed_boxes:
                    self._visualize_final_stacking()
            
            return Status(Condition.Success)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            # Create status to pass to handle_fault
            return Status(Condition.Fault, err_msg=str(e), err_type=type(e))
                
    def calculate_packing_efficiency(self):
        """Calculate and report the volumetric efficiency of the packing"""
        if not self.placed_boxes:
            print("No boxes placed, cannot calculate efficiency")
            return
        
        # Calculate total volume of all boxes
        box_volume = sum(box['dimensions'][0] * box['dimensions'][1] * box['dimensions'][2] 
                        for box in self.placed_boxes)
        
        # Calculate bounding volume of all boxes
        min_coords = np.array([float('inf'), float('inf'), float('inf')])
        max_coords = np.array([float('-inf'), float('-inf'), float('-inf')])
        
        for box in self.placed_boxes:
            pos = box['position']
            dims = box['dimensions']
            
            min_coords = np.minimum(min_coords, 
                                [pos[0] - dims[0]/2, pos[1] - dims[1]/2, pos[2] - dims[2]/2])
            max_coords = np.maximum(max_coords, 
                                [pos[0] + dims[0]/2, pos[1] + dims[1]/2, pos[2] + dims[2]/2])
        
        bounding_volume = np.prod(max_coords - min_coords)
        
        # Calculate efficiency
        efficiency = box_volume / bounding_volume if bounding_volume > 0 else 0
        
        print("\n=== PACKING EFFICIENCY ===")
        print(f"Total box volume: {box_volume:.6f} cubic meters")
        print(f"Bounding volume: {bounding_volume:.6f} cubic meters")
        print(f"Volumetric efficiency: {efficiency:.2%}")
        
        # Store for reporting
        self.packing_efficiency = efficiency
    
    def simple_box_placement(self):
        """Simple, deterministic box placement algorithm"""
        # Ensure we have a segmented pallet
        if not hasattr(self, 'pallet_pcd') or len(self.pallet_pcd.points) < 10:
            print("Error: No pallet detected")
            return []
        
        # Get pallet dimensions
        pallet_points = np.asarray(self.pallet_pcd.points)
        pallet_min = np.min(pallet_points, axis=0)
        pallet_max = np.max(pallet_points, axis=0)
        pallet_dimensions = pallet_max - pallet_min
        pallet_height = np.mean(pallet_points[:, 2])
        
        print(f"Pallet dimensions: {pallet_dimensions[0]:.4f}m x {pallet_dimensions[1]:.4f}m")
        print(f"Pallet height: {pallet_height:.4f}m")
        
        # Convert box sizes from cm to meters (direct conversion)
        meter_boxes = []
        for box in self.box_sizes:
            # Check if box is already in meters
            if max(box) < 0.1 :# Likely already in meters if below 0.5
                meter_box = box
            else:
                # Convert from cm to m
                meter_box = (box[0]*0.001, box[1]*0.001, box[2]*0.001)
            meter_boxes.append(meter_box)
        
        print(f"Box sizes in meters: {meter_boxes}")
        
        # Initialize placement tracking
        placed_boxes = []
        current_layer = 0
        current_x = pallet_min[0] + 0.005  # Start with small margin
        current_y = pallet_min[1] + 0.005
        layer_height = pallet_height
        
        # Place each box
        for i, box_size in enumerate(meter_boxes):
            width, depth, height = box_size
            
            # Check if box fits in current row
            if current_x + width > pallet_max[0] - 0.005:
                # Move to next row
                current_x = pallet_min[0] + 0.005
                current_y += depth + 0.005
            
            # Check if box fits in current layer
            if current_y + depth > pallet_max[1] - 0.005:
                # Move to next layer
                current_layer += 1
                current_x = pallet_min[0] + 0.005
                current_y = pallet_min[1] + 0.005
                
                # Update layer height to the max height of boxes in previous layer
                if placed_boxes:
                    max_prev_height = 0
                    for box in placed_boxes:
                        if box['layer'] == current_layer - 1:
                            box_top = box['position'][2] + box['dimensions'][2]/2
                            max_prev_height = max(max_prev_height, box_top)
                    layer_height = max_prev_height
            
            # Calculate position (box center)
            box_x = current_x + width/2
            box_y = current_y + depth/2
            box_z = layer_height + height/2
            
            # Create placement
            placement = {
                'position': np.array([box_x, box_y, box_z]),
                'dimensions': box_size,
                'layer': current_layer,
                'id': i+1
            }
            
            # Add to placed boxes
            placed_boxes.append(placement)
            
            # Update current_x for next box
            current_x += width + 0.005  # 5mm spacing between boxes
            
            print(f"Placed box {i+1} at {placement['position']} (layer {current_layer})")
        
        return placed_boxes

    def _find_optimal_box_placement(self, box_size):
        """Find the best placement for a new box, prioritizing side-by-side on pallet first"""
        # 1. First try to place the box on the pallet/table
        if len(self.placed_boxes) == 0:
            print("Finding first box placement on table...")
            return self._find_center_table_placement(box_size)
        
        # 2. Try adjacent placement on pallet surface before stacking
        print("Finding adjacent placement on table...")
        table_placements = self._find_adjacent_table_placements(box_size)
        if table_placements:
            best_table = max(table_placements, key=lambda x: x['score'])
            print(f"Found adjacent placement on table with score {best_table['score']:.4f}")
            return best_table
        
        # 3. Try stacking only if no good side-by-side placements
        print("Table surface full, trying to stack...")
        stacking_options = self._find_stacking_options(box_size)
        if stacking_options:
            best_stack = max(stacking_options, key=lambda x: x['score'])
            print(f"Found stacking option with score {best_stack['score']:.4f}")
            return best_stack
        
        # 4. No good placement found
        print("No valid placement found")
        return None

    def _visualize_current_stacking(self):
        """Visualize the current box stacking state with layer information"""
        # Create a copy of the original point cloud
        vis_pcd = o3d.geometry.PointCloud(self.processed_pcd)
        
        # Add box markers
        geometries = [vis_pcd]
        
        # Add layer grid visualization (optional)
        for layer_index, layer in enumerate(self.layers):
            # Different color for each layer's boxes
            layer_colors = [
                [1, 0, 0],    # Layer 0: Red
                [0, 1, 0],    # Layer 1: Green
                [0, 0, 1],    # Layer 2: Blue
                [1, 1, 0],    # Layer 3: Yellow
                [1, 0, 1],    # Layer 4: Magenta
                [0, 1, 1],    # Layer 5: Cyan
            ]
            color = layer_colors[layer_index % len(layer_colors)]
            
            # Add boxes for this layer
            for box in layer['boxes']:
                box_marker = self.create_box_marker(box['position'], box['dimensions'], color)
                geometries.append(box_marker)
        
        # Show the visualization
        o3d.visualization.draw_geometries(geometries, window_name="Layer-Based Box Stacking")

    def _visualize_placement_strategy(self):
        """Visualize the placement strategy with proximity heatmap"""
        if not hasattr(self, 'table_pcd') or len(self.table_pcd.points) == 0:
            return
        
        # Create visualization geometries
        geometries = [self.processed_pcd]
        
        # Add existing boxes
        for box in self.placed_boxes:
            box_marker = self.create_box_marker(
                box['position'],
                box['dimensions'],
                [1, 0, 0]  # Red for placed boxes
            )
            geometries.append(box_marker)
        
        # Add object centers if available
        if hasattr(self, 'detected_object_centers') and self.detected_object_centers:
            for center in self.detected_object_centers:
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
                sphere.translate(center)
                sphere.paint_uniform_color([0, 1, 0])  # Green for object centers
                geometries.append(sphere)
        
        # Visualize
        o3d.visualization.draw_geometries(
            geometries,
            window_name="Placement Strategy Visualization"
        )

    def visualize_collision_check(self, new_box, existing_boxes, collision_detected):
        """Visualize collision detection for debugging"""
        # Create visualization geometries
        geometries = []
        
        # Add table surface for reference
        if hasattr(self, 'table_pcd') and self.table_pcd is not None:
            table_vis = o3d.geometry.PointCloud(self.table_pcd)
            table_vis.paint_uniform_color([0.8, 0.8, 0.8])  # Light gray
            geometries.append(table_vis)
        
        # Add existing boxes in blue
        for box in existing_boxes:
            box_marker = self.create_box_marker(
                box['position'],
                box['dimensions'],
                [0, 0, 1]  # Blue for existing boxes
            )
            geometries.append(box_marker)
        
        # Add new box in green (no collision) or red (collision)
        new_box_color = [1, 0, 0] if collision_detected else [0, 1, 0]  # Red if collision, green if not
        new_box_marker = self.create_box_marker(
            new_box['position'],
            new_box['dimensions'],
            new_box_color
        )
        geometries.append(new_box_marker)
        
        # Visualize
        o3d.visualization.draw_geometries(
            geometries,
            window_name=f"Collision Check: {'Collision Detected' if collision_detected else 'No Collision'}"
        )

    def create_box_marker(self, position, dimensions, color=[1, 0, 0]):
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

    def _visualize_final_stacking(self):
        """Visualize the final stacking result"""
        # Create a copy of the original point cloud
        vis_pcd = o3d.geometry.PointCloud(self.processed_pcd)
        
        # Add box markers with different colors
        geometries = [vis_pcd]
        
        colors = {
            'stacked': [1, 0, 0],    # Red
            'adjacent': [0, 1, 0],   # Green
            'on_table': [0, 0, 1]    # Blue
        }
        
        for i, box in enumerate(self.placed_boxes):
            placement_type = box.get('placement_type', 'on_table')
            color = colors.get(placement_type, [0.5, 0.5, 0.5])
            
            box_marker = create_box_marker(box['position'], box['dimensions'], color)
            geometries.append(box_marker)
        
        # Show the visualization
        o3d.visualization.draw_geometries(geometries, window_name="Final Box Stacking Result")
        
        # Save the visualization to a file
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1024, height=768)
        for geom in geometries:
            vis.add_geometry(geom)
        
        # Adjust view
        vis.get_render_option().background_color = [1, 1, 1]
        vis.get_render_option().point_size = 2
        
        # Capture and save
        image_path = os.path.join(self.output_dir, "final_stacking.png")
        vis.capture_screen_image(image_path)
        vis.destroy_window()
        
        print(f"Saved visualization to {image_path}")

    def end(self) -> tuple[Status, dict]:
        """Finalize the box stacking optimization and return results.
        
        Returns:
            tuple: Status and output dictionary with stacking results
        """
        # Prepare output dictionary with results
        outputs = {
            'placed_boxes': [
                {
                    'position': box['position'].tolist(),
                    'dimensions': box['dimensions'],
                    'placement_type': box.get('placement_type', 'unknown'),
                    'score': box.get('score', 0.0)
                }
                for box in self.placed_boxes
            ],
            'total_boxes_placed': len(self.placed_boxes),
            'total_boxes_attempted': len(self.box_sizes),
            'output_dir': self.output_dir
        }
        
        # Save results to a text file for easier access
        if self.placed_boxes:
            results_path = os.path.join(self.output_dir, "stacking_results.txt")
            try:
                with open(results_path, "w") as f:
                    f.write(f"Input file: {self.ply_file_path}\n")
                    f.write(f"Successfully placed {len(self.placed_boxes)} out of {len(self.box_sizes)} boxes\n\n")
                    
                    for i, box in enumerate(self.placed_boxes):
                        f.write(f"Box {i+1}:\n")
                        f.write(f"  Dimensions: {box['dimensions'][0]:.3f} x {box['dimensions'][1]:.3f} x {box['dimensions'][2]:.3f} m\n")
                        f.write(f"  Position: [{box['position'][0]:.3f}, {box['position'][1]:.3f}, {box['position'][2]:.3f}]\n")
                        f.write(f"  Placement type: {box.get('placement_type', 'unknown')}\n")
                        f.write(f"  Stability score: {box.get('score', 0):.4f}\n\n")
                
                print(f"Saved results to {results_path}")
            except Exception as e:
                print(f"Warning: Could not save results file: {str(e)}")
        
        print("Box stacking optimization routine completed successfully.")
        print(f"Placed {len(self.placed_boxes)} out of {len(self.box_sizes)} boxes.")
        return Status(Condition.Success), outputs

    def handle_fault(self, prev_status=None):
        """Handle faults that occur during box stacking optimization."""
        # Create default status if none provided
        if prev_status is None:
            prev_status = Status(
                Condition.Fault,
                err_msg="Unknown error in box stacking optimization",
                err_type=RuntimeError
            )
        
        print(f"Box stacking optimization fault: {prev_status.err_msg}")
        
        # Return empty dictionary
        return prev_status, {}
    
    def _find_center_table_placement(self, box_size):
        """Find placement for first box at center of table"""
        if len(self.table_pcd.points) == 0:
            return None
            
        # Get table bounding box
        bbox = self.table_pcd.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        
        # Create test position
        test_position = center.copy()
        table_points = np.asarray(self.table_pcd.points)
        
        # Ensure box is on the table
        test_position = self.ensure_surface_contact(test_position, table_points, box_size)
        
        return {
            'position': test_position,
            'dimensions': box_size,
            'score': 0.95,  # High score for center placement
            'placement_type': 'on_table'
        }

    def _find_adjacent_table_placements(self, box_size):
        """Find placements adjacent to existing boxes on table"""
        if len(self.table_pcd.points) == 0:
            return []
        
        adjacent_options = []
        table_points = np.asarray(self.table_pcd.points)
        
        # Get table boundaries
        bbox = self.table_pcd.get_axis_aligned_bounding_box()
        min_bound = bbox.min_bound
        max_bound = bbox.max_bound
        
        # Create a grid of positions to try
        grid_size = 5
        x_range = np.linspace(min_bound[0] + box_size[0]/2, max_bound[0] - box_size[0]/2, grid_size)
        y_range = np.linspace(min_bound[1] + box_size[1]/2, max_bound[1] - box_size[1]/2, grid_size)
        
        for x in x_range:
            for y in y_range:
                # Skip positions too close to existing boxes
                test_position = np.array([x, y, 0])  # Z will be adjusted
                
                # Adjust height to ensure contact with table
                test_position = self.ensure_surface_contact(test_position, table_points, box_size)
                
                test_box = {
                    'position': test_position,
                    'dimensions': box_size
                }
                
                # Skip if collision
                if self.check_collision(test_box, self.placed_boxes):
                    continue
                
                # Calculate score based on:
                # 1. Distance from table edges
                edge_dist_x = min(x - min_bound[0], max_bound[0] - x) / box_size[0]
                edge_dist_y = min(y - min_bound[1], max_bound[1] - y) / box_size[1]
                edge_score = min(1.0, min(edge_dist_x, edge_dist_y))
                
                # 2. Distance from other boxes (prefer somewhat close but not too close)
                box_distance_score = 0.8  # Default
                if self.placed_boxes:
                    min_dist = float('inf')
                    optimal_dist = (box_size[0] + box_size[1]) / 2  # Optimal distance
                    
                    for box in self.placed_boxes:
                        ex_pos = box['position']
                        dist = np.sqrt((x - ex_pos[0])**2 + (y - ex_pos[1])**2)
                        min_dist = min(min_dist, dist)
                    
                    # Score higher when close to optimal distance
                    box_distance_score = max(0.1, 1.0 - abs(min_dist - optimal_dist) / optimal_dist)
                
                # Combined score (prioritize table coverage)
                score = 0.5 * edge_score + 0.5 * box_distance_score
                
                adjacent_options.append({
                    'position': test_position,
                    'dimensions': box_size,
                    'score': score,
                    'placement_type': 'on_table'
                })
        
        return adjacent_options

    def _find_stacking_options(self, box_size):
        """Find options for stacking on top of existing boxes"""
        stacking_options = []
        
        # Sort boxes by stability score (more stable boxes are better for stacking)
        stable_boxes = sorted(
            self.placed_boxes, 
            key=lambda b: b.get('score', 0),
            reverse=True
        )
        
        for existing_box in stable_boxes:
            # Position for stacking
            stack_pos = existing_box['position'].copy()
            
            # Adjust for different heights
            existing_height = existing_box['dimensions'][2]
            stack_pos[2] += (existing_height + box_size[2]) / 2
            
            # Create test box
            test_box = {
                'position': stack_pos,
                'dimensions': box_size
            }
            
            # Skip if collision with other boxes
            if self.check_collision(test_box, self.placed_boxes):
                continue
            
            # Calculate stability score
            ex_width, ex_depth = existing_box['dimensions'][0], existing_box['dimensions'][1]
            new_width, new_depth = box_size[0], box_size[1]
            
            # Calculate overlap percentages
            width_overlap = min(ex_width, new_width) / max(ex_width, new_width)
            depth_overlap = min(ex_depth, new_depth) / max(ex_depth, new_depth)
            
            # Overall stability score
            stability = width_overlap * depth_overlap
            
            # Only consider stable stacking
            if stability >= 0.7:
                stacking_options.append({
                    'position': stack_pos,
                    'dimensions': box_size,
                    'score': 0.8 * stability,  # Lower than side-by-side but higher for more stable stacks
                    'placement_type': 'stacked',
                    'reference_box': existing_box
                })
        
        return stacking_options

    def ensure_surface_contact(self, position, surface_points, box_size):
        """Ensure the box is in contact with the surface"""
        # Find the highest point near the box footprint
        box_min_x = position[0] - box_size[0]/2
        box_max_x = position[0] + box_size[0]/2
        box_min_y = position[1] - box_size[1]/2
        box_max_y = position[1] + box_size[1]/2
        
        # Add a small margin
        margin = 0.01
        footprint_mask = (
            (surface_points[:, 0] >= box_min_x - margin) &
            (surface_points[:, 0] <= box_max_x + margin) &
            (surface_points[:, 1] >= box_min_y - margin) &
            (surface_points[:, 1] <= box_max_y + margin)
        )
        
        under_box_points = surface_points[footprint_mask]
        
        if len(under_box_points) == 0:
            print("WARNING: No surface points found under box")
            return position
        
        # Get highest point under box
        max_z = np.max(under_box_points[:, 2])
        
        # Adjust position so bottom of box is at max_z
        adjusted_position = position.copy()
        adjusted_position[2] = max_z + box_size[2]/2
        
        print(f"Adjusted box height from {position[2]:.4f} to {adjusted_position[2]:.4f}")
        return adjusted_position

    def check_collision(self, new_box, existing_boxes, tolerance=0.001):
        """
        Check if a new box would collide with existing boxes.
        
        Args:
            new_box (dict): Box to check with 'position' and 'dimensions' keys
            existing_boxes (list): List of existing boxes with 'position' and 'dimensions'
            tolerance (float): Safety margin to prevent boxes from touching
            
        Returns:
            bool: True if collision detected, False otherwise
        """
        # Extract new box information
        new_pos = np.array(new_box['position'])
        new_dims = np.array(new_box['dimensions'])
        
        # Calculate new box bounds
        new_min = new_pos - new_dims/2 - tolerance
        new_max = new_pos + new_dims/2 + tolerance
        
        for existing_box in existing_boxes:
            # Skip comparison with self (important when updating positions)
            if existing_box is new_box:
                continue
                
            # Extract existing box information
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
                return True
        
        return False

    def _calculate_proximity_to_objects(self, position):
        """Calculate proximity to detected objects"""
        if not hasattr(self, 'objects_pcd') or len(self.objects_pcd.points) == 0:
            return 0.5  # Neutral score if no objects
        
        # Get object points
        object_points = np.asarray(self.objects_pcd.points)
        
        # If we have clustering, use object centers
        if hasattr(self, 'detected_object_centers') and self.detected_object_centers:
            centers = self.detected_object_centers
            
            # Find closest object center
            min_dist = float('inf')
            for center in centers:
                dist = np.sqrt((position[0] - center[0])**2 + 
                            (position[1] - center[1])**2)
                min_dist = min(min_dist, dist)
            
            # Optimal distance should be small but not zero
            # (we want to place near but not on top of objects)
            optimal_dist = 0.03  # 3cm from object center
            
            if min_dist < 0.01:  # Too close
                return 0.5
            elif min_dist < optimal_dist:
                return 1.0 - (min_dist / optimal_dist - 1.0)**2
            else:
                return max(0.0, 1.0 - (min_dist - optimal_dist) / 0.1)
        
        # If no clustering, use all object points
        else:
            # Take XY coordinates only
            object_xy = object_points[:, :2]
            position_xy = position[:2]
            
            # Find distance to closest object point
            distances = np.sqrt(np.sum((object_xy - position_xy)**2, axis=1))
            min_dist = np.min(distances) if len(distances) > 0 else float('inf')
            
            # Score based on proximity (1.0 = close, 0.0 = far)
            return max(0.0, 1.0 - min_dist / 0.1)

    def _calculate_centrality_score(self, position):
        """Calculate how central a position is on the table surface"""
        if not hasattr(self, 'table_pcd') or len(self.table_pcd.points) == 0:
            return 0.5
            
        # Get table center
        bbox = self.table_pcd.get_axis_aligned_bounding_box()
        center = bbox.get_center()
        extent = bbox.get_extent()
        
        # Calculate normalized distance from center (0.0=center, 1.0=edge)
        dx = abs(position[0] - center[0]) / (extent[0]/2)
        dy = abs(position[1] - center[1]) / (extent[1]/2)
        
        # Convert to a score where 1.0 is center, 0.0 is edge
        distance = max(dx, dy)  # Use max to be conservative
        centrality = 1.0 - min(1.0, distance)
        
        return centrality

    def _calculate_box_to_box_proximity(self, position, box_size):
        """Calculate proximity score to existing boxes"""
        if not self.placed_boxes:
            return 0.5  # Neutral if no boxes placed yet
        
        # Calculate how close this position is to existing boxes
        min_distance = float('inf')
        for box in self.placed_boxes:
            box_pos = box['position']
            # Only consider XY distance (same layer)
            distance = np.sqrt((position[0] - box_pos[0])**2 + 
                            (position[1] - box_pos[1])**2)
            
            # Calculate minimum possible distance for boxes to touch
            min_touch_distance = (box_size[0] + box['dimensions'][0])/2 + \
                                (box_size[1] + box['dimensions'][1])/2
            
            # Normalized distance (1.0 = touching, >1.0 = gap)
            norm_distance = distance / min_touch_distance
            min_distance = min(min_distance, norm_distance)
        
        # Score highest when boxes are touching or very close
        if min_distance < 1.05:  # Allow small gap
            return 1.0
        else:
            return max(0.0, 1.0 - (min_distance - 1.05))

    def initialize_first_layer(self):
        """Initialize the first layer based on pallet dimensions only"""
        # Override pallet dimensions consistently
        fixed_width = 0.2  # 20cm
        fixed_depth = 0.2  # 20cm
        
        if hasattr(self, 'pallet_pcd') and len(self.pallet_pcd.points) > 0:
            # Calculate pallet height
            pallet_points = np.asarray(self.pallet_pcd.points)
            pallet_height = np.mean(pallet_points[:, 2])
            
            # Get pallet center (use detected center, not hardcoded)
            min_bound = np.min(pallet_points, axis=0)
            max_bound = np.max(pallet_points, axis=0)
            pallet_center = (min_bound + max_bound) / 2
        else:
            print("Warning: No pallet detected, using default values")
            pallet_height = -0.14  # Use value from previous runs
            pallet_center = np.array([0, 0, pallet_height])
        
        # Use fixed dimensions with detected height
        x_min = pallet_center[0] - fixed_width/2 + 0.01  # 1cm margin
        x_max = pallet_center[0] + fixed_width/2 - 0.01
        y_min = pallet_center[1] - fixed_depth/2 + 0.01
        y_max = pallet_center[1] + fixed_depth/2 - 0.01
        
        print(f"Initializing layer with fixed pallet dimensions: {fixed_width}m x {fixed_depth}m")
        print(f"Pallet center: {pallet_center}")
        
        # Create first layer
        self.layers.append({
            'index': 0,
            'height': pallet_height,
            'bounds': [x_min, x_max, y_min, y_max],
            'boxes': [],
            'occupied_cells': set(),
            'grid_positions': []
        })
        
        self.current_layer_index = 0
        return True

    # def create_next_layer(self):
    #     """Create a new layer on top of the current one"""
    #     if not self.layers:
    #         return self.initialize_first_layer()
        
    #     current_layer = self.layers[self.current_layer_index]
        
    #     # Calculate new layer height based on boxes in current layer
    #     if not current_layer['boxes']:
    #         print("Cannot create new layer - current layer has no boxes")
    #         return False
        
    #     # Find maximum height of boxes in current layer
    #     max_box_top = 0
    #     for box in current_layer['boxes']:
    #         box_top = box['position'][2] + box['dimensions'][2]/2
    #         max_box_top = max(max_box_top, box_top)
        
    #     # Create new layer with same boundaries but higher
    #     new_layer = {
    #         'index': len(self.layers),
    #         'height': max_box_top,
    #         'bounds': current_layer['bounds'].copy(),
    #         'boxes': [],
    #         'occupied_cells': set(),
    #         'grid_positions': []
    #     }
        
    #     self.layers.append(new_layer)
    #     self.current_layer_index = len(self.layers) - 1
    #     print(f"Created new layer {self.current_layer_index} at height {max_box_top:.4f}")
        
    #     return True

    def create_next_layer(self):
        """Create a new layer on top of the current one with proper height"""
        if not self.layers:
            return self.initialize_first_layer()
        
        current_layer = self.layers[self.current_layer_index]
        
        # Calculate new layer height based on boxes in current layer
        if not current_layer['boxes']:
            print("Cannot create new layer - current layer has no boxes")
            return False
        
        # Find maximum height of boxes in current layer
        max_box_top = float('-inf')
        for box in current_layer['boxes']:
            box_top = box['position'][2] + box['dimensions'][2]/2
            max_box_top = max(max_box_top, box_top)
        
        # Create new layer with same boundaries but higher
        new_layer = {
            'index': len(self.layers),
            'height': max_box_top,  # This is crucial - set to top of highest box
            'bounds': current_layer['bounds'].copy(),
            'boxes': [],
            'occupied_cells': set(),
            'grid_positions': []
        }
        
        print(f"Created new layer {len(self.layers)} at height {max_box_top:.6f}")
        
        self.layers.append(new_layer)
        self.current_layer_index = len(self.layers) - 1
        return True

    def get_layer_grid(self, layer_index=None):
        """Generate grid positions for a layer with proper spacing between boxes"""
        if layer_index is None:
            layer_index = self.current_layer_index
            
        if layer_index >= len(self.layers):
            return []
        
        layer = self.layers[layer_index]
        
        # Extract boundaries
        x_min, x_max, y_min, y_max = layer['bounds']
        
        # Calculate number of cells
        x_cells = max(1, int((x_max - x_min) / self.grid_resolution))
        y_cells = max(1, int((y_max - y_min) / self.grid_resolution))
        
        # Set minimum spacing between boxes if not already defined
        if not hasattr(self, 'box_spacing'):
            self.box_spacing = 0.002  # 2mm spacing between boxes
        
        # Create set of occupied cells, including margins around existing boxes
        occupied_cells = set()
        
        # Mark cells used by existing boxes with a margin
        for box in layer['boxes']:
            pos = box['position']
            dims = box['dimensions']
            
            # Calculate grid coordinates with margin
            min_i = max(0, int(((pos[0] - dims[0]/2 - self.box_spacing) - x_min) / self.grid_resolution))
            max_i = min(x_cells-1, int(((pos[0] + dims[0]/2 + self.box_spacing) - x_min) / self.grid_resolution))
            min_j = max(0, int(((pos[1] - dims[1]/2 - self.box_spacing) - y_min) / self.grid_resolution))
            max_j = min(y_cells-1, int(((pos[1] + dims[1]/2 + self.box_spacing) - y_min) / self.grid_resolution))
            
            # Mark all cells in the footprint plus margin as occupied
            for i in range(min_i-1, max_i+2):  # Add extra margin
                for j in range(min_j-1, max_j+2):
                    if 0 <= i < x_cells and 0 <= j < y_cells:  # Ensure within bounds
                        occupied_cells.add((i, j))
        
        # Update layer's occupied cells
        layer['occupied_cells'].update(occupied_cells)
        
        # Only consider unoccupied cells
        grid_positions = []
        for i in range(x_cells):
            for j in range(y_cells):
                if (i, j) not in occupied_cells:
                    x = x_min + (i + 0.5) * self.grid_resolution
                    y = y_min + (j + 0.5) * self.grid_resolution
                    grid_positions.append({
                        'position': np.array([x, y, layer['height']]),
                        'grid_pos': (i, j)
                    })
        
        # Store grid positions for visualization
        layer['grid_positions'] = grid_positions
        
        print(f"Generated {len(grid_positions)} valid grid positions for layer {layer_index}")
        return grid_positions

    def get_robot_coordinates(self):
        """Get box positions in robot coordinate system"""
        robot_coordinates = []
        
        # Using identity transformation for now as requested
        for i, box in enumerate(self.placed_boxes):
            # Create a dictionary with all needed information
            robot_box = {
                'id': i+1,
                'position': box['position'].tolist(),
                'dimensions': box['dimensions'],
                'placement_type': box.get('placement_type', 'unknown'),
                'orientation': [0, 0, 0]  # Assume boxes aligned with coordinate axes
            }
            
            robot_coordinates.append(robot_box)
        
        return robot_coordinates