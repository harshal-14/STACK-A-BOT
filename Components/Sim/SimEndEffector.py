from ..EndEffector import EndEffector
import pybullet as p

class SimEndEffector(EndEffector):
    """Spoofing of simulated End Effector. 

    Because there is no simulated version of a vacuum end effector in pybullet, 
    we have to get creative about how to simulate its behavior...\n
    SimEndEffector is a singleton object and can be refered to via `SingletonRegistry.get_singleton(EndEffector)`. 
        
    Attributes:
        a (type): example attribute
        b (type): example attribute
    """
    # TODO: Implement
    def __init__(self):
        pass

    def bringup(self, **kwargs) -> int:
        return self.connect(kwargs=kwargs)

    def connect(self, **kwargs) -> int:
        return 0
    
    def disconnect(self, **kwargs) -> int:
        return 0
    
    def get_status(self) -> int:
        return 0
    
    def set_mode(self, on:int):
        pass