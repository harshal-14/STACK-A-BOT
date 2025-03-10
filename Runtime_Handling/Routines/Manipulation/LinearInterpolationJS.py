import numpy as np
import time

from .. ..Utilities import S_TO_NS
from ..Routine import Routine
from ...Status import Status, Condition
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator

class LinearInterpolationJS(Routine):

    def __init__(self, dst_q: np.ndarray, travel_time:float):
        self.dst_q = dst_q
        self.travel_time = travel_time # seconds
        self.manip_ref = get_singleton(Manipulator)

        self.init_q:np.ndarray
        self.init_time:float

    def init(self, prev_outputs, parameters = None) -> Status:
        self.init_q = self.manip_ref.get_joint_values()
        self.init_time = time.time_ns()
        return Status(Condition.Success)
    
    def loop(self) -> Status:
        # gives us a scalar value from [0,1] defining where along the path we should be aiming to go. 
        time_delta = (time.time_ns() - self.init_time) / (self.travel_time * S_TO_NS) 
        if time_delta > 1.0:
            return Status(Condition.Success)

        q_diff = self.dst_q - self.init_q
        target_q = q_diff * time_delta + self.init_q
        self.manip_ref.go_to(target_q)

        return Status(Condition.In_Progress)
    
    def end(self) -> tuple[Status, dict]:
        self.manip_ref.stop()
        return Status(Condition.Success), None
    
    def handle_fault(self, prev_status) -> tuple[Status, dict]:
        return super().handle_fault(prev_status)