from ..Components.SingletonRegistry import SingletonMeta
import pybullet as p
import pybullet_data

#TODO: @Chirag @Harshal @Raval, Someone with better pybullet knowledge pls fill in this Routine... this is my best guess as to what we need

class SimEnvironment(metaclass=SingletonMeta):

    def __init__(self, urdf_path: str, time_step: float):
        self.urdf_path = urdf_path
        self.time_step = time_step
        self.planeID = -1
        self.box_IDs = []

    def start_enviornment(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")
        print("spawning boxes")
        self.spawn_box(0)
        self.spawn_box(1, position=[0.3, -0.1, 0.03])
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(True)

    def spawn_box(self, box_num:int, position: list[float] = [0.3, 0.0, 0.05]):
        box_id = p.loadURDF(self.urdf_path + f"box{box_num}.urdf", basePosition=position)
        self.box_IDs.append(box_id)

    def stop_environment(self):
        pass