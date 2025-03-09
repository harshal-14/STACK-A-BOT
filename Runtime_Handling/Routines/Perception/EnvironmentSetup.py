from Routine import Routine
from Status import Status, Condition
import pybullet as p
import pybullet_data


#TODO: @Chirag @Harshal @Raval, Someone with better pybullet knowledge pls fill in this Routine... this is my best guess as to what we need

#NOTE: How do we want to handle the simulator? It needs to step in time to progress, how have people been handling this in their operations... 
class EnvironmnetSetup(Routine):

    def __init__(self):
        self.time_step = 1e-3
        pass

    def init(self, prev_outputs, parameters = None) -> Status:
        p.connect(p.GUI)
        return Status(Condition.Success) 
    
    def loop(self) -> Status:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(fixedTimeStep=self.time_step, numSolverIterations=100, numSubSteps=10)
        p.setRealTimeSimulation(True)
        planeId = p.loadURDF("plane.urdf")
        return Status(Condition.Success)
    
    def end(self) -> tuple[Status, dict]:
        """Not sure what to add here if anything..."""
        return Status(Condition.Success), None
    
    def fault_handler(self)-> tuple[Status, dict]:
        return None, None