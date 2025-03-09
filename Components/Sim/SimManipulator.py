from ..Manipulator import Manipulator 
import numpy as np
import pybullet as p

class SimManipulator(Manipulator):

    def __init__(self):
        print("SimManipulator")
        self.boxID = -1
        pass

    def connect(self, **kwargs) -> int:
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxId = p.loadURDF(kwargs["urdf"],startPos, startOrientation, useFixedBase=1)
        return 0

    def bringup(self, **kwargs) -> int:
        return 0

    def disconnet(self, **kwargs) -> int:
        return 0

    def go_to(self, q_array: np.ndarray) -> int:
        return 0
    
    def get_joint_values(self) -> np.ndarray:
        return np.zeros((1,1))