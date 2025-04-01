#!/usr/bin/env python3
"""
Module for integrating DUSt3R functionality from oak-d-dust3r-simplified.py
"""

import cv2
import numpy as np
import torch
import open3d as o3d
import depthai as dai
import os
import time
from datetime import datetime
from pathlib import Path

# Directly importing from the original script's imports
from dust3r.utils.image import load_images
from dust3r.model import AsymmetricCroCo3DStereo
from dust3r.image_pairs import make_pairs
from dust3r.inference import inference
from dust3r.cloud_opt import global_aligner, GlobalAlignerMode

def create_oak_pipeline():
    """Create and configure the OAK-D Lite pipeline for RGB image capture"""
    pipeline = dai.Pipeline()
    
    # Define RGB camera
    rgb_cam = pipeline.create(dai.node.ColorCamera)
    rgb_out = pipeline.create(dai.node.XLinkOut)
    rgb_out.setStreamName("rgb")
    
    # RGB camera setup
    rgb_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb_cam.setPreviewSize(640, 400)  # Set preview size explicitly
    rgb_cam.setInterleaved(False)
    rgb_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    
    # Link camera to output
    rgb_cam.preview.link(rgb_out.input)
    
    return pipeline

def capture_images_interactive(self, num_angles=4, output_dir='.'):
    """Capture images from OAK-D Lite interactively with simplified window handling."""
    if not self.device or not self.rgb_queue:
        raise RuntimeError("Camera not connected")
    
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(output_dir, f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)
    print(f"Images will be saved to: {session_dir}")
    
    image_paths = []
    
    try:
        print(f"\nWe'll capture {num_angles} different views of your scene.")
        print("For best results, move around the object between captures to get different angles.")
        
        # Kill all OpenCV windows before starting
        cv2.destroyAllWindows()
        
        # Create a SINGLE window for the entire process
        WINDOW_NAME = "Camera Capture"
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 640, 480)
        
        for angle in range(num_angles):
            print(f"\n=== Preparing to capture angle {angle+1}/{num_angles} ===")
            print("Live preview started. Position the camera for a good view.")
            print("Press 'c' when ready to capture, or 'q' to quit.")
            
            capture_done = False
            while not capture_done:
                # Get frame
                rgb_data = self.rgb_queue.get()
                rgb_frame = np.array(rgb_data.getData()).reshape((rgb_data.getHeight(), rgb_data.getWidth(), 3))
                
                # Create display frame
                display_frame = rgb_frame.copy()
                
                # Add instructions text
                cv2.putText(display_frame, f"Angle {angle+1}/{num_angles} - Press 'c' to capture", 
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show the frame
                cv2.imshow(WINDOW_NAME, display_frame)
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('c'):
                    # Take a new frame for the actual capture
                    rgb_data = self.rgb_queue.get()
                    rgb_frame = np.array(rgb_data.getData()).reshape((rgb_data.getHeight(), rgb_data.getWidth(), 3))
                    
                    # Save the image
                    image_path = os.path.join(session_dir, f"view{angle+1:02d}.jpg")
                    cv2.imwrite(image_path, rgb_frame)
                    image_paths.append(image_path)
                    
                    # Show "Captured" message
                    confirm_frame = rgb_frame.copy()
                    cv2.putText(confirm_frame, "CAPTURED", (50, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                    cv2.imshow(WINDOW_NAME, confirm_frame)
                    cv2.waitKey(1000)
                    
                    capture_done = True
                    
                elif key == ord('q'):
                    print("Capture process cancelled by user.")
                    cv2.destroyAllWindows()
                    return []
        
        # Clean up
        cv2.destroyAllWindows()
        print(f"\nSuccessfully captured {len(image_paths)} angles!")
        
    except Exception as e:
        print(f"Error during image capture: {str(e)}")
        import traceback
        traceback.print_exc()
        cv2.destroyAllWindows()
        return []
        
    return image_paths
            
def process_images_with_dust3r(image_paths, 
                              model_name="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt",
                              image_size=512,
                              output_dir="."):
    """Process captured images with DUSt3R to generate a point cloud"""
    if len(image_paths) < 2:
        print("Need at least 2 images for DUSt3R reconstruction")
        return None
    
    print(f"Processing {len(image_paths)} images with DUSt3R")
    
    try:
        # Ensure output directory exists
        os.makedirs(output_dir, exist_ok=True)
        
        # Load DUSt3R model
        print(f"Loading DUSt3R model: {model_name}")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {device}")
        model = AsymmetricCroCo3DStereo.from_pretrained(model_name).to(device)
        model.eval()
        
        # Load images using DUSt3R's utility
        print("Loading images...")
        images = load_images(image_paths, size=image_size)
        
        # Create pairs
        print("Creating image pairs...")
        pairs = make_pairs(images, scene_graph='complete', prefilter=None, symmetrize=True)
        print(f"Created {len(pairs)} image pairs")
        
        # Run inference
        print("Running DUSt3R inference...")
        output = inference(pairs, model, device, batch_size=1)
        
        # Create scene for global alignment
        print("Setting up global alignment...")
        scene = global_aligner(output, device=device, mode=GlobalAlignerMode.PointCloudOptimizer)
        
        # Optimize alignment
        print("Optimizing global alignment...")
        scene.compute_global_alignment(init="mst", niter=300, schedule='cosine', lr=0.01)
        
        # Extract point cloud
        print("Extracting point cloud...")
        pts3d = scene.get_pts3d()
        confidence_masks = scene.get_masks()
        
        # Combine points from all views
        print("Combining points from all views...")
        valid_points = []
        for i in range(len(pts3d)):
            conf_mask = confidence_masks[i].cpu().numpy()
            points = pts3d[i].detach().cpu().numpy()[conf_mask]
            if len(points) > 0:
                valid_points.append(points)
        
        if not valid_points:
            print("No valid points generated. Try different camera angles.")
            return None
            
        # Combine all points
        combined_points = np.vstack(valid_points)
        print(f"Generated point cloud with {len(combined_points)} points")
        
        # Save point cloud
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        pcd_path = os.path.join(output_dir, f"dust3r_point_cloud_{timestamp}.ply")
        
        # Create Open3D point cloud for saving
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(combined_points)
        o3d.io.write_point_cloud(pcd_path, pcd)
        print(f"Point cloud saved to {pcd_path}")
        
        return combined_points
        
    except Exception as e:
        print(f"Error in DUSt3R processing: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def create_point_cloud_from_dust3r(num_angles=4, 
                                  model_name="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt",
                                  image_size=512,
                                  output_dir=".", 
                                  existing_images=None):
    """Complete pipeline to capture images and process with DUSt3R"""
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Use existing images or capture new ones
    if existing_images:
        if isinstance(existing_images, str) and os.path.isdir(existing_images):
            # If a directory is provided, find images in it
            print(f"Looking for images in: {existing_images}")
            image_extensions = ['.jpg', '.jpeg', '.png']
            image_paths = []
            for ext in image_extensions:
                image_paths.extend([str(p) for p in Path(existing_images).glob(f'*{ext}')])
            image_paths.sort()
            print(f"Found {len(image_paths)} images")
        else:
            # Assume list of image paths
            image_paths = existing_images
    else:
        # Capture new images
        print("Capturing new images...")
        image_paths = capture_images_interactive(num_angles, output_dir)
    
    if not image_paths:
        print("No images available for processing")
        return None
    
    # Process images with DUSt3R
    point_cloud = process_images_with_dust3r(image_paths, model_name, image_size, output_dir)
    
    return point_cloud

def visualize_point_cloud(point_cloud, output_path=None):
    """Visualize the point cloud using Open3D"""
    if len(point_cloud) == 0:
        print("No points to visualize")
        return None
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    
    # Colorize points by height (z-coordinate) for better visualization
    points_array = np.asarray(pcd.points)
    if len(points_array) > 0:
        z_values = points_array[:, 2]
        min_z, max_z = np.min(z_values), np.max(z_values)
        
        # Normalize z values to [0,1] range
        if max_z > min_z:
            normalized_z = (z_values - min_z) / (max_z - min_z)
        else:
            normalized_z = np.zeros_like(z_values)
        
        # Create colormap (blue for low, red for high)
        colors = np.zeros((len(point_cloud), 3))
        colors[:, 0] = normalized_z  # Red channel
        colors[:, 2] = 1 - normalized_z  # Blue channel
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # Save point cloud if output path is provided
    if output_path:
        try:
            o3d.io.write_point_cloud(output_path, pcd)
            print(f"Point cloud saved to {output_path}")
        except Exception as e:
            print(f"Error saving point cloud: {str(e)}")
    
    # Try to visualize (skip if in a headless environment)
    try:
        o3d.visualization.draw_geometries([pcd])
    except Exception as e:
        print(f"Could not display visualization: {str(e)}")
        print("If running in a headless environment, you can load the saved PLY file later for visualization.")
    
    return pcd

def convert_point_cloud_to_o3d(point_cloud):
    """Convert numpy point cloud to Open3D format"""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    return pcd

# If run directly, demonstrate functionality
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="DUSt3R 3D Reconstruction")
    parser.add_argument("--output-dir", type=str, default="./dust3r_output",
                        help="Output directory for saved files")
    parser.add_argument("--num-angles", type=int, default=4,
                        help="Number of angles to capture")
    parser.add_argument("--image-dir", type=str, default=None,
                        help="Directory with existing images (instead of capturing)")
    parser.add_argument("--model", type=str, 
                        default="naver/DUSt3R_ViTLarge_BaseDecoder_512_dpt",
                        help="DUSt3R model name or path")
    
    args = parser.parse_args()
    
    # Run the complete pipeline
    if args.image_dir:
        print(f"Using existing images from: {args.image_dir}")
        point_cloud = create_point_cloud_from_dust3r(
            existing_images=args.image_dir,
            output_dir=args.output_dir,
            model_name=args.model
        )
    else:
        print("Starting interactive capture process...")
        point_cloud = create_point_cloud_from_dust3r(
            num_angles=args.num_angles,
            output_dir=args.output_dir,
            model_name=args.model
        )
    
    if point_cloud is not None:
        visualize_point_cloud(point_cloud)
        print("Process completed successfully")
    else:
        print("Failed to generate point cloud")