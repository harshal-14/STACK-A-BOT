"""
This module contains the MoveAndCapture class, which is responsible for moving the robot to a specified position and capturing an image.
"""

import os
import time
import datetime as datetime

from ..Routine import Routine
from ...Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator
from ....Components.Camera import Camera
from ....World.Geometry import Pose
from ....Utilities import S_TO_NS, inverse_quaternion, multiply_quaternion, exponentiate_quaternion

class MoveAndCapture(Routine):
    """Routine that moves the robot to a position and then captures an image.
    
    Combines the functionality of LinearInterpolationTS with automatic camera capture
    when the target position is reached.
    
    Attributes:
        dst_pose (Pose): Target end effector pose
        travel_time (float): Time to travel to the position in seconds
        output_dir (str): Directory to save captured images
        position_name (str): Optional name for this position (used for image filename)
        manip_ref (Manipulator): Reference to manipulator instance
        camera_ref (Camera): Reference to camera instance
        init_pose (Pose): Initial pose at start of movement
        init_time (float): Time when movement started
        capture_complete (bool): Whether image capture has completed
    """

    def __init__(self, dst_pose: Pose, travel_time: float, output_dir: str = './output', position_name: str = None):
        """Initializes the MoveAndCapture routine.
        
        Args:
            dst_pose (Pose): Target end effector pose
            travel_time (float): Time to travel to the position in seconds
            output_dir (str): Directory to save captured images
            position_name (str): Optional name for this position (used for image filename)
        """
        self.dst_pose = dst_pose
        self.travel_time = travel_time
        self.output_dir = output_dir
        self.position_name = position_name
        self.capture_complete = False
        
        # Will be initialized in init()
        self.manip_ref = None
        self.camera_ref = None
        self.init_pose = None
        self.init_time = None

    def init(self):
        """Initializes the routine by getting references to the manipulator and camera. Store initial pose & tinme"""
        self.manip_ref = get_singleton(Manipulator)
        self.camera_ref = get_singleton(Camera)
        self.init_pose = self.manip_ref.FK_Solver(self.manip_ref.get_joint_values())
        self.init_time = time.time_ns()
        self.capture_complete = False

        os.makedirs(self.output_dir, exist_ok=True)

        return Status(Condition.SUCCESS, "MoveAndCapture initialized successfully.")
    
    def loop(self) -> Status:
        """Main loop of the routine. Moves the robot and captures an image when the target position is reached.
        
        Returns:
            Status: The status of the routine (SUCCESS, FAILURE, or RUNNING)
        """

        time_delta = (time.time_ns() - self.init_time) / (self.travel_time *S_TO_NS)
        if time_delta >= 1.0:
            # We've reached the destination
            if not self.capture_complete:
                # Capture an image
                try:
                    filename = self.position_name
                    if not filename:
                        filename = f"pos_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    
                    # Use the non-interactive capture method
                    self.camera_ref.capture_image(filename=filename, output_dir=self.output_dir)
                    self.capture_complete = True
                except Exception as e:
                    print(f"Error capturing image: {e}")
                    # Continue even if capture fails
                    self.capture_complete = True
            
            return Status(Condition.Success)
        
        # Linear interpolation for position
        point_different = self.dst_pose.point.to_np() - self.init_pose.point.to_np()
        target_point = point_different * time_delta + self.init_pose.point.to_np()
        
        # SLERP for rotation
        q_1 = self.init_pose.orientation.to_quat()
        q_2 = self.dst_pose.orientation.to_quat()
        
        a = multiply_quaternion(q_2, inverse_quaternion(q_1))
        b = exponentiate_quaternion(a, time_delta)
        
        target_quat = multiply_quaternion(b, q_1)
        
        # Create and apply target pose
        from ....World.Geometry import RotMatrix
        target_rot = RotMatrix.from_quat(target_quat)
        self.manip_ref.move_ts(Pose(target_rot, target_point))
        
        return Status(Condition.In_Progress)
    
    def end(self) -> tuple[Status, dict]:
        """Stop the manipulator and return status and any outputs."""
        self.manip_ref.stop()
        
        return Status(Condition.Success), {
            "position_captured": self.position_name if self.position_name else "unnamed_position"
        }
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        return Status(prev_status.cond, prev_status.err_msg, prev_status.err_type), {}
