from ..EndEffector import EndEffector

class HwEndEffector(EndEffector):

    def __init__(self):
        pass

    def bringup(self, **kwargs) -> int:
        return 0

    def connect(self, **kwargs) -> int:
        return 0
    
    def disconnet(self, **kwargs) -> int:
        return 0

    def get_status(self) -> int:
        return 0
    
    def set_mode(self, on:int):
        pass
