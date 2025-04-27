"""
This module contains the MoveAndCaptureJS class, which moves the robot to a specified joint position and captures an image.
"""

import os
import time
from datetime import datetime
import numpy as np

from ..Routine import Routine
from ...Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator
from ....Components.Camera import Camera
from ....Utilities import S_TO_NS, joint_array_sanitizer

class MoveAndCaptureJS(Routine):
    """Routine that moves the robot to a joint position and then captures an image.
    
    Combines the functionality of LinearInterpolationJS with automatic camera capture
    when the target position is reached.
    
    Attributes:
        dst_q (np.ndarray): Target joint angles
        travel_time (float): Time to travel to the position in seconds
        output_dir (str): Directory to save captured images
        position_name (str): Optional name for this position (used for image filename)
        manip_ref (Manipulator): Reference to manipulator instance
        camera_ref (Camera): Reference to camera instance
        init_q (np.ndarray): Initial joint angles at start of movement
        init_time (float): Time when movement started
        capture_complete (bool): Whether image capture has completed
    """

    def __init__(self, dst_q: np.ndarray, travel_time: float, output_dir: str = './output', position_name: str = None):
        """Initializes the MoveAndCaptureJS routine.
        
        Args:
            dst_q (np.ndarray): Target joint angles
            travel_time (float): Time to travel to the position in seconds
            output_dir (str): Directory to save captured images
            position_name (str): Optional name for this position (used for image filename)
        """
        self.dst_q = dst_q
        self.travel_time = travel_time
        self.output_dir = output_dir
        self.position_name = position_name
        self.capture_complete = False
        
        # Will be initialized in init()
        self.manip_ref = None
        self.camera_ref = None
        self.init_q = None
        self.init_time = None

    def init(self, prev_outputs, parameters = None) -> Status:
        """Initializes the routine by getting references to the manipulator and camera. Store initial joints & time"""
        self.manip_ref = get_singleton(Manipulator)
        self.camera_ref = get_singleton(Camera)
        self.init_q = self.manip_ref.get_joint_values()
        self.init_time = time.time_ns()
        self.capture_complete = False

        os.makedirs(self.output_dir, exist_ok=True)

        return Status(Condition.Success)
    
    def loop(self) -> Status:
        """Main loop of the routine. Moves the robot and captures an image when the target position is reached.
        
        Returns:
            Status: The status of the routine (SUCCESS, FAILURE, or RUNNING)
        """

        time_delta = (time.time_ns() - self.init_time) / (self.travel_time * S_TO_NS)
        if time_delta >= 1.0:
            # We've reached the destination broooo!!!!!
            if not self.capture_complete:
                
                try:
                    filename = self.position_name
                    if not filename:
                        filename = f"pos_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    
                    self.camera_ref.capture_image(filename=filename, output_dir=self.output_dir)
                    self.capture_complete = True
                except Exception as e:
                    print(f"Error capturing image: {e}")
                    # Continue even if capture fails
                    self.capture_complete = True
            
            return Status(Condition.Success)
        
        # Changed from Linear interpolation task space to joint space
        q_diff = self.dst_q - self.init_q
        target_q = q_diff * time_delta + self.init_q
        self.manip_ref.move_js(target_q)
        
        return Status(Condition.In_Progress)
    
    def end(self) -> tuple[Status, dict]:
        """Stop the manipulator and return status and any outputs."""
        self.manip_ref.stop()
        
        return Status(Condition.Success), {
            "position_captured": self.position_name if self.position_name else "unnamed_position"
        }
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        return Status(prev_status.cond, prev_status.err_msg, prev_status.err_type), {}