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
        cur_q = self.manip_ref.get_joint_values()
        fk_output = self.manip_ref.FK_Solver(cur_q)
        ik_output = self.manip_ref.IK_Solver(fk_output)
        # np.set_printoptions(precision=3, suppress=True) # just for this print
        # print(self.manip_ref._get_ee())
        if time_delta > 1.0:
            return Status(Condition.Success)
        
        point_different = self.dst_pose.point.to_np() - self.init_pose.point.to_np()
        target_point = point_different * time_delta + self.init_pose.point.to_np()

        q_1 = self.init_pose.orientation.to_quat()
        q_2 = self.dst_pose.orientation.to_quat()

        q_inv = inverse_quaternion(q_1)
        a = multiply_quaternion(q_2, q_inv)
        b = exponentiate_quaternion(a, time_delta)
        # SLERP - spherical linear interpolation - of the rotation matrix.
        target_quat = multiply_quaternion(b, q_1)
        
        # print(target_quat)
        target_rot = RotMatrix.from_quat(target_quat)
        # print(target_rot.rot)
        self.manip_ref.move_ts(Pose(target_rot, target_point))

        return Status(Condition.In_Progress)
    
    def end(self) -> tuple[Status, dict]:
        """Stops moving manipulator to end motion."""
        self.manip_ref.stop()
        return Status(Condition.Success), None
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        return super().handle_fault(prev_status)