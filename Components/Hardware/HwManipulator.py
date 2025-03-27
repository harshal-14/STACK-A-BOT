from ..Manipulator import Manipulator 
import numpy as np
from ...World.Geometry import Pose, Point 

class HwManipulator(Manipulator):
    """Low-level integration & communication with Manipulator.

    HwManipulator is an arduino connected via serial. High level requests must be translated into GCode and sent via serial to the arduino. 
    For more info, look at http://thor.angel-lm.com/documentation/firmware/. and https://reprap.org/wiki/G-code\n
    HwManipulator is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
    
    Attributes:
        a (type): example attribute
        b (type): example attribute 2
    """
    # TODO: Implement
    def __init__(self):
        print("HwManipulator")
        pass

    def connect(self, **kwargs) -> int:
        return 0

    def bringup(self, **kwargs) -> int:
        return 0

    def disconnect(self, **kwargs) -> int:
        return 0

    def move_js(self, q_array: np.ndarray):
        pass

    def move_ts(self, pose:Pose):
        pass
    
    def get_joint_values(self) -> np.ndarray:
        return np.zeros((1))