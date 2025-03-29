import numpy as np
import time

from .. ..Utilities import S_TO_NS, inverse_quaternion, multiply_quaternion, exponentiate_quaternion
from ..Routine import Routine
from ...Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator
from ....World.Geometry import Pose, Point, RotMatrix

class LinearInterpolationTS(Routine):
    """Basic point-to-point movement Routine in Task Space. 

        Target position is determined by percentage of time passed between travel_time and time at init().

        Attributes:
            dst_pose (Pose): desired final end effector position in Joint space. Should be a (6,1) numpy array in RADIANS
            travel_time (float): Time taken in seconds from initial and destination position
            manip_ref (Manipulator): reference to manipulator instance
            init_pose (Pose): joint angles captured during init() call
            init_time (float): EPOCH time in nano-seconds during init() call 
    """

    def __init__(self, dst_pose: Pose, travel_time:float):
        self.dst_pose = dst_pose
        self.travel_time = travel_time # seconds
        
        self.manip_ref:Manipulator
        self.init_pose:Pose
        self.init_time:float

    def init(self, prev_outputs, parameters = None) -> Status:
        """Gets reference to the manipulaotr obj, captures ititial time and position for movement."""
        self.manip_ref = get_singleton(Manipulator)
        self.init_pose = self.manip_ref.FK_Solver(self.manip_ref.get_joint_values())
        self.init_time = time.time_ns()
        return Status(Condition.Success)
    
    def loop(self) -> Status:
        """Linearly interpolates target position based on current time. Returns Success once we have traveled for travel_time seconds.
            May want to change behavior to check if motion was actually successful...
        """
        # Scalar value from [0,1] rep. distance along path. 
        time_delta = (time.time_ns() - self.init_time) / (self.travel_time * S_TO_NS)
        if time_delta > 1.0:
            return Status(Condition.Success)
        
        # target position in Task space is interpolated linearly
        point_different = self.dst_pose.point.to_np() - self.init_pose.point.to_np()
        target_point = point_different * time_delta + self.init_pose.point.to_np()

        """ Target rotation is done via SLERP - spherical linear interpolation - of the rotation matrix. 
            Along a time vector t in [0, 1], the target quat vector can be found via the following equation
                (q_2 * q_1^(-1))^t * q_1
            The code below is an expansion of that formula in a readable format. """
        q_1 = self.init_pose.orientation.to_quat() # [W X Y Z]
        q_2 = self.dst_pose.orientation.to_quat()

        a = multiply_quaternion(q_2, inverse_quaternion(q_1)) # a = (q_2 * q_1^(-1))
        b = exponentiate_quaternion(a, time_delta) # b = a^t
        
        target_quat = multiply_quaternion(b, q_1) # b * q_1

        # We translate quaternion back into a rotation matrix and move to new ee_pose
        target_rot = RotMatrix.from_quat(target_quat)
        self.manip_ref.move_ts(Pose(target_rot, target_point))

        return Status(Condition.In_Progress)
    
    def end(self) -> tuple[Status, dict]:
        """Stops moving manipulator to end motion."""
        self.manip_ref.stop()
        return Status(Condition.Success), None
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        return super().handle_fault(prev_status)