from ..EndEffector import EndEffector
from .HwInterface import HwInterface
from ..SingletonRegistry import get_singleton

class HwEndEffector(EndEffector):
    """Low-level integration & communication with vacuum pump and switch. 

    HwEndEffector is a singleton object and can be refered to via `SingletonRegistry.get_singleton(EndEffector)`. 
    
    Attributes:
        status (int): refers to status of pump. Either pulling vacuum (1) or passive (0)
        hw_interface(HWInterface): the serial connection for which we send status commands to
    """
    def __init__(self):
        self.status = False
        self.hw_interface:HwInterface

    def bringup(self, **kwargs) -> int:
        self.hw_interface = HwInterface()
        if self.connect() == -1:
            return -1
        # set pump OFF and wait for "ok" response
        retstr = self.hw_interface.tx_rx("M3 S0")
        if "error" in retstr or "ALARM" in retstr:
            raise RuntimeError("HWManipulator.update_joint_values() returned something other than 'ok'.\n"
                              f"potential fault msg: {retstr}")
        return 0
    
    def connect(self, **kwargs) -> int:
       return self.hw_interface.connect_device()

    def disconnect(self, **kwargs) -> int:
        if self.hw_interface.connected:
            self.set_mode(False)
        return 0
    def get_status(self) -> int:
        return self.status
    
    def set_mode(self, new_status:bool):
        if new_status:
            retstr = self.hw_interface.tx_rx("M3 S1000")
        else:
            retstr = self.hw_interface.tx_rx("M3 0")
        if retstr == "ok":
            self.status = new_status
        else:
            print(f"Error encountered: {retstr}")
