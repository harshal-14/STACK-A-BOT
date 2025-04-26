from ..Manipulator import Manipulator 
from .HwInterface import HwInterface
from ..SingletonRegistry import get_singleton
import numpy as np
from ...World.Geometry import Pose
from ...Utilities import joint_array_sanitizer

import threading
import time

MAX_ATTEMPTS = 5

class HwManipulator(Manipulator):
    """Low-level integration & communication with Manipulator.

    HwManipulator is an arduino connected via serial. High level requests must be translated into GCode and sent via serial to the arduino. 
    For more info, look at http://thor.angel-lm.com/documentation/firmware/. and https://reprap.org/wiki/G-code\n
    HwManipulator is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
    
    Attributes:
        hw_interface(HWInterface): The serial connection for which we send status commands.
        current_position (np.ndarray): (6,1) Position of joints.
    """
    def __init__(self, ):
        self.hw_interface:HwInterface
        self.current_position = None
        # Threading objects
        self.position_thread = threading.Thread(None, self.update_joint_values, "Position thread")
        self._pos_lock = threading.Lock()
        self.stop_thread = False

    def __del__(self):
        self.stop()

    def stop(self):
        if self.hw_interface.connected:
            self.hw_interface.tx_rx("!")
    
    def bringup(self, **kwargs) -> int:
        self.hw_interface = get_singleton(HwInterface)
        if not self.connect():
            return -1
        # wait until we get a successful readout of our current position
        position_iter_attempts = 0
        while position_iter_attempts < MAX_ATTEMPTS:
            with self._pos_lock:
                if(self.current_position is not None): break
            position_iter_attempts+=1
            time.sleep(0.5)

        if position_iter_attempts == MAX_ATTEMPTS:
            print("Unable to read position from Manipulator")
            return -1
        return 0
    
    def connect(self, **kwargs) -> int:
        if not self.hw_interface.connect_device():
            return -1
        self.position_thread.start()
        return 0

    def disconnect(self, **kwargs) -> int:
        self.stop_thread = True
        retval = 0
        try:
            self.position_thread.join(timeout=2)
        except RuntimeError as e:
            print(e)
            retval = -1
        self.stop_thread = False
        return retval

    def move_js(self, q_array: np.ndarray):
        """Sends a uninterpolated move cmd to the hw_interface for movement of all 6 joints"""

        qs = joint_array_sanitizer(q_array)
        # Rapid uninterpolated movement        B and C must be equivalent...
        message = ( f"G0 A{qs[0]:.3f} B{qs[1]:.3f} C{qs[1]:.3f} D{qs[2]:.3f}"
                      f" X{qs[3]:.3f} Y{qs[4]:.3f} Z{qs[5]:.3f}\n")
        
        retstr = self.hw_interface.tx_rx(message)
        if "error" in retstr or "ALARM" in retstr:
            raise RuntimeError("HWManipulator.move_js() returned something other than 'ok'.\n"
                               f"potential fault msg: {retstr}")

    def move_ts(self, pose:Pose):
        ikpy_joints = self.IK_Solver(pose)
        self.move_js(ikpy_joints)

    def update_joint_values(self):
        """ Function to query grbl for current position, 
            \nAs reccomended by GRBL documentation, this runs at 10Hz max, any faster yields diminshing returns.
        """
        print("Starting position thread")
        while True:
            if not self.hw_interface.connected or self.stop_thread: break
            retstr = self.hw_interface.tx_rx("?")
            print(retstr)
            if "error" in retstr or "ALARM" in retstr:
                raise RuntimeError("HWManipulator.update_joint_values() returned something other than 'ok'.\n"
                               f"potential fault msg: {retstr}")
            else:
                raw_data = retstr[1:][:-1].split(",")
                qs = np.array([[float(raw_data[1][5:])], [float(raw_data[2])], [float(raw_data[4])], 
                               [float(raw_data[5])], [float(raw_data[6])], [float(raw_data[7])]])
                qs *= (np.pi/180.0)
                with self._pos_lock:
                    self.current_position = qs
            time.sleep(0.1) # sleep for 100ms 
    
    def get_joint_values(self) -> np.ndarray:
        """ Function returns the current position of the end_effector as updated by the position thread """
        retval = None
        with self._pos_lock:
            retval = self.current_position
        return retval
    