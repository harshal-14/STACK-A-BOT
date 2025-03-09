from Routine import Routine
from Status import Status, Condition
from .. ..Components.SingletonRegistry import get_singleton
from .. ..Components.Manipulator import Manipulator

class Homing(Routine):

    def __init__(self):

        self.manip_ref = get_singleton(Manipulator)
        self.homing_delta = 1e-2
        pass

    def init(self, prev_outputs, parameters = None) -> Status:
        return Status(Condition.Success)
    
    def loop(self):
        ret_status = Status(Condition.Success)
        cur_pos = self.manip_ref.get_joint_values()
        for i in range(cur_pos.shape()[0]):
            if abs(cur_pos[i]) > self.homing_delta
        
        
        
        return ret_status
    
    def end(self):
        return super().end()
    
    def fault_handler(self):
        return super().fault_handler()