from Routine import Routine
from .. ..World.SimEnvironment import SimEnvironment
from Status import Status, Condition
import pybullet as p

class EnvironmnetSetup(Routine):

    def __init__(self, time_step):
        self.time_step = time_step
        pass

    def init(self, prev_outputs, parameters = None) -> Status:
        p.connect(p.GUI)
        self.sim_env = SimEnvironment(time_step=1e-3)
        return Status(Condition.Success) 
    
    def loop(self) -> Status:
        self.sim_env.start_enviornment()
        return Status(Condition.Success)
    
    def end(self) -> tuple[Status, dict]:
        """Not sure what to add here if anything..."""
        return Status(Condition.Success), None
    
    def fault_handler(self)-> tuple[Status, dict]:
        return None, None