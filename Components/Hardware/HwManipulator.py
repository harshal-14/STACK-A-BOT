from ..Manipulator import Manipulator 
import numpy as np

class HwManipulator(Manipulator):

    def __init__(self):
        print("HwManipulator")
        pass

    def connect(self, **kwargs) -> int:
        return 0

    def bringup(self, **kwargs) -> int:
        return 0

    def disconnet(self, **kwargs) -> int:
        return 0

    def go_to(self, q_array: np.ndarray) -> int:
        return 0
    
    def get_joint_values(self) -> np.ndarray:
        return np.zeros((1))