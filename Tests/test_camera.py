'''
Script to check the HwCamera class and its functions
'''

import os
import sys

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

from ..Components.Hardware.HwCamera import HwCamera
from ..Components.SingletonRegistry import update_singleton_registry
from ..Components.Camera import Camera
import numpy as np
import cv2

def main():
    camera = HwCamera()
    
    update_singleton_registry(Camera, camera)
    print("Connecting to camera...")
    if camera.connect() != 0:
        print("Failed to connect to camera. Exiting.")
        return 1
    
    try:
        print("Testing RGB image capture...")
        rgb_frame = camera.get_RGB_image()
        print(f"RGB frame shape: {rgb_frame.shape}")
        
        rgb_display = np.transpose(rgb_frame, (1, 2, 0))
        rgb_display = (rgb_display * 255).astype(np.uint8)
        cv2.imshow("RGB Image", rgb_display)
        cv2.waitKey(1000)
        
        print("Testing depth image capture...")
        depth_frame = camera.get_depth_data()
        # print(f"Depth frame shape: {depth_frame.shape}")
        
        depth_display = depth_frame[0]  # Get first channel
        depth_colormap = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1000)
        
        print("Testing mono image capture...")
        left_frame, right_frame = camera.get_mono_images()
        print(f"Left frame shape: {left_frame.shape}")
        print(f"Right frame shape: {right_frame.shape}")
        
        left_display = (left_frame[0] * 255).astype(np.uint8)
        right_display = (right_frame[0] * 255).astype(np.uint8)
        cv2.imshow("Left Mono", left_display)
        cv2.imshow("Right Mono", right_display)
        
        print("Press any key to close windows and continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        print("\nCamera test completed successfully!")
        
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