from ..EndEffector import EndEffector

class HwEndEffector(EndEffector):
    """Low-level integration & communication with vacuum pump and switch. 

    HwEndEffector is a singleton object and can be refered to via `SingletonRegistry.get_singleton(EndEffector)`. 
    
    Attributes:
        status (int): refers to status of pump. Either pulling vacuum (1) or passive (0)
    """
    # TODO: Implement
    def __init__(self):
        print("HwEndEffector")
        self.status = False
        pass

    def bringup(self, **kwargs) -> int:
        return self.connect(kwargs=kwargs)

    def connect(self, **kwargs) -> int:
        return 0
    
    def disconnet(self, **kwargs) -> int:
        return 0

    def get_status(self) -> int:
        return 0
    
    def set_mode(self, on:int):
        pass
