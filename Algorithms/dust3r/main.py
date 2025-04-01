import sys
import argparse
import time
import os
from pathlib import Path
import numpy as np
import open3d as o3d

# Import the StabilizedVoxelGrid3D class
try:
    from improved_voxels import StabilizedVoxelGrid3D
except ImportError:
    print("Error: Could not import StabilizedVoxelGrid3D.")
    print("Make sure the improved_voxels.py file is in the same directory.")
    sys.exit(1)

# Import DUSt3R module if available
try:
    import dust3r_module
    DUST3R_AVAILABLE = True
except ImportError:
    print("Warning: DUSt3R module not available. Install required dependencies for better reconstruction.")
    DUST3R_AVAILABLE = False

# Import segmentation module if available
try:
    from Algorithms.VoxelGrid.segment_point_cloud import segment_point_cloud, visualize_segments
    SEGMENTATION_AVAILABLE = True
except ImportError:
    print("Warning: Point cloud segmentation module not available.")
    print("Place segment_point_cloud.py in the same directory for better object isolation.")
    SEGMENTATION_AVAILABLE = False

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Stabilized 3D Occupancy Grid Mapping')
    
    parser.add_argument('--voxel-size', type=float, default=0.05,
                        help='Size of voxels in meters (default: 0.05)')
    
    parser.add_argument('--confidence-threshold', type=int, default=120,
                        help='Depth confidence threshold (0-255, default: 120)')
    
    parser.add_argument('--output-dir', type=str, default='maps',
                        help='Directory to save maps (default: maps)')
    
    parser.add_argument('--mode', type=str, 
                        choices=['live', 'record', 'dust3r-capture', 'dust3r-from-images'],
                        default='live',
                        help='Mode: live visualization, record data, or use DUSt3R')
    
    parser.add_argument('--record-interval', type=int, default=30,
                        help='Interval in seconds to save maps when in record mode (default: 30)')
    
    parser.add_argument('--max-drift', type=float, default=0.02,
                        help='Maximum allowed camera drift per frame in meters (default: 0.02)')
    
    parser.add_argument('--num-angles', type=int, default=4,
                        help='Number of angles to capture for DUSt3R (default: 4)')
    
    parser.add_argument('--dust3r-model', type=str, 
                        default="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt",
                        help='DUSt3R model name or path')
    
    parser.add_argument('--image-size', type=int, default=512,
                        help='Size to resize images for DUSt3R processing (default: 512)')
    
    parser.add_argument('--image-dir', type=str, default=None,
                        help='Directory containing images for DUSt3R reconstruction')
    
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize the occupancy grid after creation')
    
    parser.add_argument('--threshold', type=float, default=0.65,
                        help='Occupancy probability threshold (0-1, default: 0.65)')
    
    parser.add_argument('--segment', action='store_true',
                        help='Apply advanced segmentation to isolate objects')
    
    parser.add_argument('--input-ply', type=str, default=None,
                        help='Input PLY file if operating directly on a point cloud')
    
    return parser.parse_args()

def ensure_output_dir(directory):
    """Ensure output directory exists."""
    path = Path(directory)
    if not path.exists():
        path.mkdir(parents=True)
    return path

def run_dust3r_mode(args, output_dir):
    """Run DUSt3R reconstruction and create occupancy grid."""
    print(f"Starting DUSt3R-based reconstruction in {args.mode} mode...")
    
    # Get point cloud from DUSt3R
    if args.input_ply:
        print(f"Using existing point cloud from {args.input_ply}")
        try:
            pcd = o3d.io.read_point_cloud(args.input_ply)
            point_cloud = np.asarray(pcd.points)
        except Exception as e:
            print(f"Error loading point cloud: {e}")
            return False
    elif args.mode == 'dust3r-capture':
        # Generate point cloud by capturing new images
        point_cloud = dust3r_module.create_point_cloud_from_dust3r(
            num_angles=args.num_angles,
            model_name=args.dust3r_model,
            image_size=args.image_size,
            output_dir=str(output_dir)
        )
    else:  # dust3r-from-images
        # Generate point cloud from existing images
        if not args.image_dir:
            print("Error: --image-dir must be specified for dust3r-from-images mode")
            return False
        
        point_cloud = dust3r_module.create_point_cloud_from_dust3r(
            existing_images=args.image_dir,
            model_name=args.dust3r_model,
            image_size=args.image_size,
            output_dir=str(output_dir)
        )
    
    if point_cloud is None or len(point_cloud) == 0:
        print("Error: Failed to generate point cloud")
        return False
    
    print(f"Successfully generated point cloud with {len(point_cloud)} points")
    
    # Create pcd object for visualization
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    
    # First visualize the raw point cloud if requested
    if args.visualize:
        print("Visualizing raw point cloud...")
        o3d.visualization.draw_geometries([pcd])
    
    # Apply advanced segmentation if requested
    if args.segment and SEGMENTATION_AVAILABLE:
        print("Applying advanced segmentation...")
        # Use a smaller voxel size for better detail
        segments = segment_point_cloud(pcd, voxel_size=args.voxel_size*0.5)
        
        # Visualize segments if requested
        if args.visualize:
            print("Visualizing segmented point cloud...")
            visualize_segments(segments)
        
        # Save segments
        timestr = time.strftime("%Y%m%d-%H%M%S")
        for i, segment in enumerate(segments):
            segment_path = os.path.join(output_dir, f"segment_{i}_{timestr}.ply")
            o3d.io.write_point_cloud(segment_path, segment)
            print(f"Saved segment {i} to {segment_path}")
        
        # Process each object segment separately
        for i, segment in enumerate(segments):
            # Skip ground plane (usually the last segment)
            if i == len(segments) - 1 and len(segments) > 1:
                print(f"Skipping segment {i} (likely ground plane)")
                continue
                
            if len(segment.points) < 100:
                print(f"Skipping segment {i} (too few points: {len(segment.points)})")
                continue
                
            print(f"Processing segment {i} with {len(segment.points)} points...")
            
            # Create voxel grid with smaller voxel size for better detail
            voxel_size = args.voxel_size
            print(f"Using voxel size: {voxel_size}m")
            
            voxel_grid = StabilizedVoxelGrid3D(voxel_size=voxel_size)
            voxel_grid.depth_confidence_threshold = args.confidence_threshold
            voxel_grid.max_drift = args.max_drift
            
            # Convert segment to numpy array
            segment_points = np.asarray(segment.points)
            
            # Process segment
            voxel_grid.process_dust3r_point_cloud(segment_points, reset_grid=True)
            
            # Save occupancy grid and mesh
            segment_occupied_points = voxel_grid.get_occupied_voxels(threshold=args.threshold)
            
            if len(segment_occupied_points) > 0:
                # Create point cloud from occupied voxels
                occupied_pcd = o3d.geometry.PointCloud()
                occupied_pcd.points = o3d.utility.Vector3dVector(segment_occupied_points)
                
                # Save as PLY
                grid_path = os.path.join(output_dir, f"segment_{i}_grid_{timestr}.ply")
                o3d.io.write_point_cloud(grid_path, occupied_pcd)
                print(f"Occupancy grid for segment {i} saved to {grid_path}")
                
                # Export as mesh
                mesh_path = os.path.join(output_dir, f"segment_{i}_mesh_{timestr}.ply")
                success = voxel_grid.export_mesh(mesh_path)
                if success:
                    print(f"Mesh for segment {i} saved to {mesh_path}")
                
                # Visualize if requested
                if args.visualize:
                    print(f"Visualizing occupancy grid for segment {i}...")
                    voxel_grid.visualize_occupancy_grid(threshold=args.threshold)
            else:
                print(f"No occupied voxels found for segment {i}")
        
        return True
    
    # Process the full point cloud if no segmentation or segmentation failed
    # Create voxel grid with smaller voxel size for better detail
    voxel_size = args.voxel_size
    print(f"Using voxel size: {voxel_size}m")
    
    voxel_grid = StabilizedVoxelGrid3D(voxel_size=voxel_size)
    voxel_grid.depth_confidence_threshold = args.confidence_threshold
    voxel_grid.max_drift = args.max_drift
    
    # Set threshold values from command line
    threshold = args.threshold
    
    print("Creating occupancy grid from point cloud...")
    voxel_grid.process_dust3r_point_cloud(point_cloud, reset_grid=True)
    
    # Save occupancy grid and mesh
    timestr = time.strftime("%Y%m%d-%H%M%S")
    
    # Get occupied voxels
    occupied_points = voxel_grid.get_occupied_voxels(threshold=threshold)
    
    if len(occupied_points) > 0:
        # Create point cloud from occupied voxels
        occupied_pcd = o3d.geometry.PointCloud()
        occupied_pcd.points = o3d.utility.Vector3dVector(occupied_points)
        
        # Save as PLY
        grid_path = os.path.join(output_dir, f"occupancy_grid_{timestr}.ply")
        o3d.io.write_point_cloud(grid_path, occupied_pcd)
        print(f"Occupancy grid saved to {grid_path}")
        
        # Export as mesh
        mesh_path = os.path.join(output_dir, f"mesh_{timestr}.ply")
        success = voxel_grid.export_mesh(mesh_path)
        if success:
            print(f"Mesh saved to {mesh_path}")
    else:
        print(f"Warning: No occupied voxels found with threshold {threshold}")
        
        # Try with a lower threshold as a fallback
        lower_threshold = threshold / 2
        print(f"Trying again with lower threshold: {lower_threshold}")
        occupied_points = voxel_grid.get_occupied_voxels(threshold=lower_threshold)
        
        if len(occupied_points) > 0:
            # Create point cloud from occupied voxels
            occupied_pcd = o3d.geometry.PointCloud()
            occupied_pcd.points = o3d.utility.Vector3dVector(occupied_points)
            
            # Save as PLY
            grid_path = os.path.join(output_dir, f"occupancy_grid_lowthresh_{timestr}.ply")
            o3d.io.write_point_cloud(grid_path, occupied_pcd)
            print(f"Occupancy grid (with lower threshold) saved to {grid_path}")
    
    # Visualize if requested
    if args.visualize:
        print(f"Visualizing occupancy grid with threshold {threshold}...")
        voxel_grid.visualize_occupancy_grid(threshold=threshold)
        
        # If no points with standard threshold, try with lower threshold
        if len(occupied_points) == 0:
            print(f"Trying visualization with lower threshold {lower_threshold}...")
            voxel_grid.visualize_occupancy_grid(threshold=lower_threshold)
    
    return True

def main():
    """Main entry point."""
    # Parse command line arguments
    args = parse_arguments()
    
    # Ensure output directory exists
    output_dir = ensure_output_dir(args.output_dir)
    
    try:
        print("Starting stabilized 3D occupancy grid mapping...")
        print(f"Voxel size: {args.voxel_size}m")
        print(f"Mode: {args.mode}")
        
        if args.mode in ['dust3r-capture', 'dust3r-from-images'] or args.input_ply:
            # Check if DUSt3R module is available
            if not DUST3R_AVAILABLE and not args.input_ply:
                print("Error: DUSt3R module not available")
                return 1
                
            # Run DUSt3R mode
            success = run_dust3r_mode(args, output_dir)
            return 0 if success else 1
        
        else:  # live or record mode
            # Create the voxel grid with desired parameters
            voxel_grid = StabilizedVoxelGrid3D(
                voxel_size=args.voxel_size
            )
            
            # Set parameters from command line arguments
            voxel_grid.depth_confidence_threshold = args.confidence_threshold
            voxel_grid.max_drift = args.max_drift
            
            # Run based on mode
            if args.mode == 'live':
                voxel_grid.run_with_visualization()
            elif args.mode == 'record':
                print(f"Recording mode: Saving maps every {args.record_interval} seconds")
                # The recording function would be implemented here
                print("Recording mode not implemented in this simplified version")
                return 1
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())#!/usr/bin/env python3