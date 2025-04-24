#!/usr/bin/env python3
"""
Test script for DUSt3R and VoxelGrid integration with the robot system.
"""
import os
import sys
import argparse

# Add the project root to the Python path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from ..Components.Hardware.HwCamera import HwCamera
from ..Components.SingletonRegistry import update_singleton_registry
from ..Components.Camera import Camera
from ..Runtime_Handling.RoutineScheduler import RoutineScheduler
from ..Runtime_Handling.Routines.Perception.DUSt3RTestRoutine import DUSt3RTestRoutine
from ..Runtime_Handling.Routines.Perception.VoxelGridTestRoutine import VoxelGridTestRoutine

def parse_args():
    parser = argparse.ArgumentParser(description="Test DUSt3R and VoxelGrid integration")
    parser.add_argument('--output-dir', type=str, default='./output',
                      help='Directory to save output files')
    parser.add_argument('--mode', choices=['dust3r', 'voxel', 'both'], default='dust3r',
                      help='Which component to test')
    parser.add_argument('--num-angles', type=int, default=4,
                      help='Number of angles to capture for DUSt3R')
    parser.add_argument('--image-dir', type=str, default=None,
                      help='Directory with existing images (instead of capturing)')
    parser.add_argument('--voxel-size', type=float, default=0.05,
                      help='Size of voxels in meters')
    parser.add_argument('--model', type=str, 
                      default="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt",
                      help='DUSt3R model name')
    parser.add_argument('--num-frames', type=int, default=5,
                      help='Number of frames to process for voxel grid')
    return parser.parse_args()

def main():
    args = parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    
    camera = HwCamera()
    
    # Register camera in singleton registry
    update_singleton_registry(Camera, camera)
    print("Connecting to camera...")
    if camera.connect() != 0:
        print("Failed to connect to camera. Exiting.")
        return 1
    
    try:
        # Set up routines
        routines = []
        
        # Add DUSt3R test if requested
        if args.mode in ['dust3r', 'both']:
            routines.append(DUSt3RTestRoutine(
                num_angles=args.num_angles,
                model_name=args.model,
                output_dir=args.output_dir,
                image_dir=args.image_dir
            ))
        
        # Add VoxelGrid test if requested
        if args.mode in ['voxel', 'both']:
            routines.append(VoxelGridTestRoutine(
                voxel_size=args.voxel_size,
                output_dir=args.output_dir,
                num_frames=args.num_frames,
                use_dust3r_cloud=(args.mode == 'both')
            ))
        print("Starting Routine Scheduler")
        print(f"Running {len(routines)} test routines")
        scheduler = RoutineScheduler(routines)
        print("\n=== Starting test routines ===")
        while scheduler.has_routines():
            scheduler.run()
        
        print("\n=== Test completed successfully ===")
        
    except Exception as e:
        print(f"Error during testing: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        print("Disconnecting from camera...")
        camera.disconnect()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())