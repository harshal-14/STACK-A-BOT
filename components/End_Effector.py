from abc import ABC, abstractmethod
from Component import Component

class End_Effector(ABC, Component):
    """Abstract representation of a suction-cup gripper on the robot end effector. 
        High-level API call are defined here without implementation specfic behavior. """
    
    @abstractmethod
    def get_status(self) -> int:
        """ Retrieves the status of the end_effector in SW.
            Returns:
                status (int): returns if the EE is currently pulling vacumm (1) or not holding vaccum (0) 
        """ 
        raise NotImplementedError("get_status() not implemented by subclass")
    
    @abstractmethod
    def set_mode(self, on:int):
        """ Toggles the suction cup based on the parameter and its current state. 
            Args:
                on (int): Requested mode for the end effector, either (1) for on, or (0) for off.
        """ 
        raise NotImplementedError("set_mode() not implemented by subclass")