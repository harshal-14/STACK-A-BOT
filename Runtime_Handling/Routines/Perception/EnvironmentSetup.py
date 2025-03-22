from ..Routine import Routine
from .. .Status import Status, Condition
from .. ..World.SimEnvironment import SimEnvironment
import pybullet as p

class EnvironmentSetup(Routine):
    """ In the event of a simulated run, creates an environment mananger object, and relevent sim objects (pallet and boxes?) 

    Attributes:
        time_step (float): time in seconds between simulation step updates.
    """
    def __init__(self, urdf_path: str, time_step: float):
        self.urdf_path = urdf_path
        self.time_step = time_step # in seconds
        pass

    def init(self, prev_outputs, parameters = None) -> Status:
        p.connect(p.GUI)
        # We unlock the Singleton Registry to enable instantiation of new singleton object. We then re-lock the registry to prevent other calls to __new__.
        # The lock condition prevents users from referencing or spawning references to uninitialized components. 
        SimEnvironment.unlock() 
        self.sim_env = SimEnvironment(self.urdf_path, time_step=self.time_step)
        SimEnvironment.lock()
        return Status(Condition.Success) 
    
    def loop(self) -> Status:
        self.sim_env.start_enviornment()
        return Status(Condition.Success)
    
    def end(self) -> tuple[Status, dict]:
        """Not sure what to add here if anything..."""
        return Status(Condition.Success), None
    
    def handle_fault(self)-> tuple[Status, dict]:
        return None, None