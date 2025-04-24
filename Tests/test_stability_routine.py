#!/usr/bin/env python3
"""
Test script for BoxStackingOptimizationRoutine with the robot system.
Analyzes a point cloud file to find optimal box stacking configurations.
"""
import os
import sys
import argparse
import numpy as np

# Add the project root to the Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from ..Runtime_Handling.RoutineScheduler import RoutineScheduler
from ..Runtime_Handling.Routines.Perception.BoxStackingOptimizationRoutine import BoxStackingOptimizationRoutine

def parse_args():
    parser = argparse.ArgumentParser(
        description="Optimize box stacking using point cloud data"
    )
    parser.add_argument("input_file", type=str, help="Path to the PLY point cloud file")
    parser.add_argument("--boxes", type=str, default="", help="Box dimensions in format 'w1,d1,h1;w2,d2,h2;...'")
    parser.add_argument("--num_boxes", type=int, default=5, help="Number of boxes to place (if --boxes not specified)")
    parser.add_argument("--default_size", type=str, default="0.2,0.15,0.1", help="Default box size (width,depth,height)")
    parser.add_argument("--no_viz", action="store_true", help="Disable visualization")
    parser.add_argument("--no_orientation_fix", action="store_true", help="Disable automatic orientation fix")
    parser.add_argument("--output_dir", type=str, default="box_stacking_output", help="Directory to save output files")
    
    return parser.parse_args()

def main():
    args = parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.input_file):
        print(f"Error: File {args.input_file} not found")
        return 1
    
    # Parse box dimensions
    box_sizes = []
    if args.boxes:
        # Parse from string format 'w1,d1,h1;w2,d2,h2;...'
        box_specs = args.boxes.split(';')
        for spec in box_specs:
            if not spec.strip():
                continue
            dims = [float(x) for x in spec.split(',')]
            if len(dims) != 3:
                print(f"Warning: Invalid box specification '{spec}', skipping")
                continue
            box_sizes.append(tuple(dims))
    else:
        # Create random variations of the default size
        default_dims = [float(x) for x in args.default_size.split(',')]
        if len(default_dims) != 3:
            default_dims = [0.2, 0.15, 0.1]  # Fallback if invalid
        
        # Create specified number of boxes with variations
        for i in range(args.num_boxes):
            # Vary dimensions by ±30%
            variation = 0.3
            varied_dims = [
                default_dims[0] * (1 + variation * (np.random.random() - 0.5)),
                default_dims[1] * (1 + variation * (np.random.random() - 0.5)),
                default_dims[2] * (1 + variation * (np.random.random() - 0.5))
            ]
            box_sizes.append(tuple(varied_dims))
    
    print(f"Using {len(box_sizes)} boxes with dimensions:")
    for i, size in enumerate(box_sizes):
        print(f"  Box {i+1}: {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m")
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Create box stacking optimization routine
    stacking_routine = BoxStackingOptimizationRoutine(
        ply_file_path=args.input_file,
        box_sizes=box_sizes,
        visualize=not args.no_viz,
        fix_orientation=not args.no_orientation_fix
    )
    
    try:
        # Set up routine scheduler with our routine
        print("Starting Routine Scheduler")
        scheduler = RoutineScheduler([stacking_routine])
        
        print("\n=== Starting box stacking optimization ===")
        try:
            while scheduler.has_routines():
                scheduler.run()
        except Exception as e:
            print(f"Error in routine scheduler: {str(e)}")
            import traceback
            traceback.print_exc()
        
        # Check if the routine completed successfully
        if hasattr(stacking_routine, "placed_boxes") and stacking_routine.placed_boxes:
            print("\n=== BOX STACKING OPTIMIZATION RESULTS ===")
            print(f"Successfully placed {len(stacking_routine.placed_boxes)} out of {len(box_sizes)} boxes")
            
            for i, box in enumerate(stacking_routine.placed_boxes):
                print(f"\nBox {i+1}:")
                print(f"  Dimensions: {box['dimensions'][0]:.3f} x {box['dimensions'][1]:.3f} x {box['dimensions'][2]:.3f} m")
                print(f"  Position: [{box['position'][0]:.3f}, {box['position'][1]:.3f}, {box['position'][2]:.3f}]")
                print(f"  Placement type: {box.get('placement_type', 'unknown')}")
                print(f"  Stability score: {box.get('score', 0):.4f}")
            
            print(f"\nResults saved to '{args.output_dir}' directory")
            return 0
        else:
            print("\n=== OPTIMIZATION COMPLETED WITHOUT FINDING SUITABLE PLACEMENTS ===")
            print("Please try:")
            print("1. Different point cloud file")
            print("2. Using --no_orientation_fix flag")
            print("3. Different box dimensions")
            print("4. Examine generated files in the output directory for troubleshooting")
            return 1
            
    except Exception as e:
        print(f"Error during box stacking optimization: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())