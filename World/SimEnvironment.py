from ..Components.SingletonRegistry import SingletonMeta
import threading
import time
import pybullet as p
import pybullet_data
import os

#TODO: @Chirag @Harshal @Raval, Someone with better pybullet knowledge pls fill in this Routine... this is my best guess as to what we need

class SimEnvironment(metaclass=SingletonMeta):

    def __init__(self, urdf_path: str, time_step: float):
        self.urdf_path = urdf_path
        self.time_step = time_step
        self.stop_cond = threading.Event()
        self.thread_obj = Sim_Thread(time_step, self.stop_cond)
        self.planeID = -1
        self.box_IDs = []

    def start_enviornment(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")
        # plane_id = p.createCollisionShape(p.GEOM_PLANE)
        # ground_id = p.createMultiBody(0, plane_id)
        print("spawning boxes")
        self.spawn_box(0)
        self.spawn_box(1, position=[0.3, -0.1, 0.03])
        p.setGravity(0, 0, -9.81)
        # Unsure if I am doing the time scaling correctly...
        p.setTimeStep(self.time_step)
        p.setPhysicsEngineParameter(fixedTimeStep=self.time_step, numSolverIterations=100, numSubSteps=10)
        p.setRealTimeSimulation(True)
        self.thread_obj.start()

    def spawn_box(self, box_num:int, position: list[float] = [0.3, 0.0, 0.05]):
        # TODO: Boxes do not seem to have a clue what gravity is. 
        box_id = p.loadURDF(self.urdf_path + f"box{box_num}.urdf", basePosition=position)
        self.box_IDs.append(box_id)

    def stop_environment(self):
        self.stop_cond.set()
        self.thread_obj.join(timeout=2.0)
        if self.thread_obj.is_alive():
            raise RuntimeWarning("Sim Thread is still alive after attempt to kill it...")

class Sim_Thread(threading.Thread):

    def __init__(self, step_time, stop_cond, group = None, target = None, name = None, args = ..., kwargs = None, *, daemon = None):
        super().__init__(group, target, name, args, kwargs, daemon=daemon)
        self.step_time = step_time
        self.stop_cond = stop_cond

    def run(self):
        """Two possible impls. sleep for step_time and then call p.stepSim(), 
            or poll time and step when diff>step_time. """
        while(not self.stop_cond.is_set()):
                p.stepSimulation()
                # time.sleep(self.step_time)