#!/usr/bin/env python3
"""
Test script for StabilityAnalysisRoutine with the robot system.
Analyzes a point cloud file to find stable box placements.
"""
import os
import sys
import argparse

# Add the project root to the Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from ..Runtime_Handling.RoutineScheduler import RoutineScheduler
from ..Runtime_Handling.Routines.Perception.StabilityAnalysisRoutine import StabilityAnalysisRoutine

def parse_args():
    parser = argparse.ArgumentParser(
        description="Test stability analysis on a point cloud file"
    )
    parser.add_argument("input_file", type=str, help="Path to the PLY point cloud file")
    parser.add_argument("--box_width", type=float, default=0.02, help="Width of the box in meters")
    parser.add_argument("--box_depth", type=float, default=0.015, help="Depth of the box in meters")
    parser.add_argument("--box_height", type=float, default=0.01, help="Height of the box in meters")
    parser.add_argument("--no_viz", action="store_true", help="Disable visualization")
    parser.add_argument("--no_orientation_fix", action="store_true", help="Disable automatic orientation fix")
    parser.add_argument("--output_dir", type=str, default="stability_output", help="Directory to save output files")
    
    return parser.parse_args()

def main():
    args = parse_args()
    
    # Check if input file exists
    if not os.path.exists(args.input_file):
        print(f"Error: File {args.input_file} not found")
        return 1
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Define box dimensions
    box_size = (args.box_width, args.box_depth, args.box_height)
    print(f"Box dimensions (width x depth x height): {box_size[0]} x {box_size[1]} x {box_size[2]} meters")
    
    # Create stability analysis routine
    stability_routine = StabilityAnalysisRoutine(
        ply_file_path=args.input_file,
        box_size=box_size,
        visualize=not args.no_viz,
        fix_orientation=not args.no_orientation_fix
    )
    
    try:
        # Set up routine scheduler with our stability routine
        print("Starting Routine Scheduler")
        scheduler = RoutineScheduler([stability_routine])
        
        print("\n=== Starting stability analysis ===")
        try:
            while scheduler.has_routines():
                scheduler.run()
        except Exception as e:
            print(f"Error in routine scheduler: {str(e)}")
            print("This may be expected if no suitable surfaces were found.")
            import traceback
            traceback.print_exc()
        
        # Check if the routine completed successfully
        if hasattr(stability_routine, "best_placement") and stability_routine.best_placement is not None:
            print("\n=== STABILITY ANALYSIS RESULTS ===")
            print(f"Best placement location: [{stability_routine.best_placement[0]:.4f}, {stability_routine.best_placement[1]:.4f}, {stability_routine.best_placement[2]:.4f}]")
            print(f"Stability score: {stability_routine.stability_score:.4f} (0-1 scale, higher is better)")
            print(f"Results saved to '{args.output_dir}' directory")
            
            # Save a simple text file with the results
            with open(os.path.join(args.output_dir, "placement_results.txt"), "w") as f:
                f.write(f"Input file: {args.input_file}\n")
                f.write(f"Box dimensions: {box_size[0]} x {box_size[1]} x {box_size[2]} meters\n")
                f.write(f"Best placement: [{stability_routine.best_placement[0]:.6f}, {stability_routine.best_placement[1]:.6f}, {stability_routine.best_placement[2]:.6f}]\n")
                f.write(f"Stability score: {stability_routine.stability_score:.6f}\n")
            
            return 0
        else:
            print("\n=== ANALYSIS COMPLETED WITHOUT FINDING SUITABLE PLACEMENT ===")
            print("Please try:")
            print("1. Different point cloud file")
            print("2. Using --no_orientation_fix flag")
            print("3. Check if the point cloud is correctly scaled")
            print("4. Examine generated files in the output directory for troubleshooting")
            return 1
            
    except Exception as e:
        print(f"Error during stability analysis: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())