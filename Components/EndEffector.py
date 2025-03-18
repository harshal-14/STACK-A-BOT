from abc import ABC, abstractmethod
from .Component import Component

class EndEffector(Component, ABC):
    """Abstract representation of a suction-cup gripper on the robot end effector. 
        High-level API call are defined here without implementation specfic behavior.\n
        EndEffector is a singleton object with one of its concrete classes being defined in its place.
        It can be refered to via `SingletonRegistry.get_singleton(EndEffector)`. 
    """
    
    # TODO: Manipulation team: Define all EE behavior here, and implement the methods in both Hw and Sim impls. 
    # We should decide if any sensors attached should be made into their own components, or defined in here...
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