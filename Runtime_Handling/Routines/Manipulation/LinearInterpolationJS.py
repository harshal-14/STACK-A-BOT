import numpy as np
import time

from .. ..Utilities import S_TO_NS
from ..Routine import Routine
from ...Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator

class LinearInterpolationJS(Routine):
    """Basic point-to-point movement Routine in Joint space. 

        Target position is determined by percentage of time passed between travel_time and time at init().

        Attributes:
            dst_q (np.ndarray): desired final end effector position in Joint space. Should be a (6,1) numpy array in RADIANS
            travel_time (float): Time taken in seconds from initial and destination position
            manip_ref (Manipulator): reference to manipulator instance
            init_q (np.ndarray): joint angles captured during init() call
            init_time (float): EPOCH time in nano-seconds during init() call 
    """

    def __init__(self, dst_q: np.ndarray, travel_time:float):
        self.dst_q = dst_q
        self.travel_time = travel_time # seconds
        
        self.manip_ref:Manipulator
        self.init_q:np.ndarray
        self.init_time:float

    def init(self, prev_outputs, parameters = None) -> Status:
        """Gets reference to the manipulaotr obj, captures ititial time and position for movement."""
        self.manip_ref = get_singleton(Manipulator)
        self.init_q = self.manip_ref.get_joint_values()
        self.init_time = time.time_ns()
        return Status(Condition.Success)
    
    def loop(self) -> Status:
        """Linearly interpolates target position based on current time. Returns Success once we have traveled for travel_time seconds.
            May want to change behavior to check if motion was actually successful...
        """
        # gives us a scalar value from [0,1] defining where along the path we should be aiming to go. 
        time_delta = (time.time_ns() - self.init_time) / (self.travel_time * S_TO_NS)
        cur_q = self.manip_ref.get_joint_values()
        fk_output = self.manip_ref.FK_Solver(cur_q)
        ik_output = self.manip_ref.IK_Solver(fk_output)
        np.set_printoptions(precision=3, suppress=True) # just for this print
        print(f"joint_pose: {cur_q.reshape((6,))},\nik_output: {ik_output}")
        if time_delta > 1.0:
            return Status(Condition.Success)

        q_diff = self.dst_q - self.init_q
        target_q = q_diff * time_delta + self.init_q
        self.manip_ref.move_js(target_q)

        return Status(Condition.In_Progress)
    
    def end(self) -> tuple[Status, dict]:
        """Stops moving manipulator to end motion."""
        self.manip_ref.stop()
        return Status(Condition.Success), None
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        return super().handle_fault(prev_status)