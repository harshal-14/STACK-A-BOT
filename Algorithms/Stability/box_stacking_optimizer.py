import numpy as np
import open3d as o3d
import os

from point_cloud_processor import (
    load_point_cloud,
    preprocess_point_cloud,
    segment_table_and_objects,
    identify_boxes,
    extract_box_surfaces
)

from box_stacking_utils import (
    evaluate_stacking_stability,
    check_collision,
    get_box_top_surface,
    get_box_side_surfaces,
    create_box_marker,
    find_best_box_placement_on_table
)

class BoxStackingOptimizer:
    def __init__(self, visualize=True, output_dir="stacking_output"):
        self.visualize = visualize
        self.output_dir = output_dir
        self.placed_boxes = []
        os.makedirs(output_dir, exist_ok=True)
        
    def process_point_cloud(self, ply_file_path, fix_orientation=True):
        """Process the point cloud and segment table and objects"""
        # Load and preprocess the point cloud
        self.raw_pcd = load_point_cloud(ply_file_path, fix_orientation)
        self.processed_pcd = preprocess_point_cloud(self.raw_pcd)
        
        # Segment table and objects
        self.table_pcd, self.objects_pcd = segment_table_and_objects(self.processed_pcd)
        
        # Identify individual boxes
        self.detected_boxes = identify_boxes(self.objects_pcd)
        
        # Extract box surfaces
        self.box_surfaces = extract_box_surfaces(self.detected_boxes)
        
        if self.visualize:
            # Visualize the segmentation
            o3d.visualization.draw_geometries([self.table_pcd, self.objects_pcd], 
                                             window_name="Segmented Point Cloud")
        
        return len(self.detected_boxes) > 0
        
    def find_optimal_box_placement(self, box_size):
        """Find the best placement for a new box"""
        print(f"\nFinding optimal placement for box size: {box_size}")
        
        # Prioritize stacking if possible
        if self.placed_boxes:
            print("Evaluating stacking options...")
            stacking_options = []
            
            # Try placing on top of existing boxes
            for existing_box in self.placed_boxes:
                # Create a new box positioned on top
                top_surface = get_box_top_surface(existing_box)
                
                # Position for stacking - center of the top surface plus half the height of the new box
                stack_pos = top_surface['position'].copy()
                stack_pos[2] += box_size[2] / 2
                
                # Create test box for stacking
                test_box = {
                    'position': stack_pos,
                    'dimensions': box_size
                }
                
                # Skip if collision with other boxes
                if check_collision(test_box, self.placed_boxes):
                    continue
                
                # Evaluate stability
                stability = evaluate_stacking_stability(existing_box, test_box)
                
                if stability['stable']:
                    stacking_options.append({
                        'position': stack_pos,
                        'dimensions': box_size,
                        'score': 0.8 + 0.2 * stability['support_ratio'],  # High base score for stacking
                        'placement_type': 'stacked',
                        'reference_box': existing_box
                    })
            
            # Choose the best stacking option if available
            if stacking_options:
                best_stacking = max(stacking_options, key=lambda x: x['score'])
                print(f"Found stable stacking option with score {best_stacking['score']:.4f}")
                return best_stacking
                
            # Try adjacent placement
            print("Evaluating adjacent placement options...")
            adjacent_options = []
            
            for existing_box in self.placed_boxes:
                # Get side surfaces
                side_surfaces = get_box_side_surfaces(existing_box)
                
                for side in side_surfaces:
                    # Position for adjacent placement
                    adj_pos = side['position'].copy()
                    
                    # Adjust position based on side type
                    if side['type'] == 'right':
                        adj_pos[0] += box_size[0] / 2
                    elif side['type'] == 'left':
                        adj_pos[0] -= box_size[0] / 2
                    elif side['type'] == 'front':
                        adj_pos[1] += box_size[1] / 2
                    elif side['type'] == 'back':
                        adj_pos[1] -= box_size[1] / 2
                    
                    # Adjust height to be on the same level as the existing box
                    adj_pos[2] = existing_box['position'][2]
                    
                    # Create test box
                    test_box = {
                        'position': adj_pos,
                        'dimensions': box_size
                    }
                    
                    # Skip if collision
                    if check_collision(test_box, self.placed_boxes):
                        continue
                    
                    # Simple score based on proximity
                    adjacent_options.append({
                        'position': adj_pos,
                        'dimensions': box_size,
                        'score': 0.7,  # Lower than stacking but higher than table
                        'placement_type': 'adjacent',
                        'reference_box': existing_box
                    })
            
            # Choose the best adjacent option if available
            if adjacent_options:
                best_adjacent = max(adjacent_options, key=lambda x: x['score'])
                print(f"Found adjacent placement option with score {best_adjacent['score']:.4f}")
                return best_adjacent
        
        # If no stacking or adjacent options, place on table
        print("Finding placement on table...")
        table_placement = find_best_box_placement_on_table(
            self.table_pcd, 
            box_size, 
            self.placed_boxes
        )
        
        if table_placement:
            print(f"Found table placement with score {table_placement['score']:.4f}")
            return table_placement
        
        print("No valid placement found")
        return None
    
    def place_boxes(self, box_sizes):
        """Place multiple boxes with different sizes"""
        placed_box_info = []
        
        # Sort boxes by volume (place largest boxes first)
        sorted_sizes = sorted(box_sizes, key=lambda size: size[0] * size[1] * size[2], reverse=True)
        
        for i, box_size in enumerate(sorted_sizes):
            print(f"\n==== PLACING BOX {i+1}/{len(sorted_sizes)} ====")
            print(f"Box dimensions: {box_size}")
            
            # Find the best placement for this box
            placement = self.find_optimal_box_placement(box_size)
            
            if placement:
                print(f"Placed box at position: {placement['position']}")
                print(f"Placement type: {placement.get('placement_type', 'unknown')}")
                
                # Add to placed boxes list
                self.placed_boxes.append(placement)
                placed_box_info.append(placement)
                
                # Visualize the current state if requested
                if self.visualize:
                    self.visualize_current_stacking()
            else:
                print(f"Could not find a suitable placement for box {i+1}")
                break
        
        # Final visualization
        if self.visualize and self.placed_boxes:
            self.visualize_final_stacking()
            
        return placed_box_info
    
    def visualize_current_stacking(self):
        """Visualize the current box stacking state"""
        # Create a copy of the original point cloud
        vis_pcd = o3d.geometry.PointCloud(self.processed_pcd)
        
        # Add box markers
        geometries = [vis_pcd]
        
        for i, box in enumerate(self.placed_boxes):
            # Use different colors based on placement type
            if box.get('placement_type') == 'stacked':
                color = [1, 0, 0]  # Red for stacked boxes
            elif box.get('placement_type') == 'adjacent':
                color = [0, 1, 0]  # Green for adjacent boxes
            else:
                color = [0, 0, 1]  # Blue for table boxes
                
            box_marker = create_box_marker(box['position'], box['dimensions'], color)
            geometries.append(box_marker)
        
        # Show the visualization
        o3d.visualization.draw_geometries(geometries, window_name="Current Box Stacking")
        
    def visualize_final_stacking(self):
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
        image_path = os.path.join(self.output_dir, "final_stacking.png")
        print(f"Saving visualization to {image_path}")