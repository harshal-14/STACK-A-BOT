from ..Manipulator import Manipulator 
import numpy as np
from ...World.Geometry import Pose, Point 
from ...Utilities import joint_array_sanitizer

import glob
import serial
import sys

class HwManipulator(Manipulator):
    """Low-level integration & communication with Manipulator.

    HwManipulator is an arduino connected via serial. High level requests must be translated into GCode and sent via serial to the arduino. 
    For more info, look at http://thor.angel-lm.com/documentation/firmware/. and https://reprap.org/wiki/G-code\n
    HwManipulator is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
    
    Attributes:
        connection (serial.Serial): Serial connection to Manipulator Arduino
        connected (bool): Status of Connection. Coupled only via calles to connect() and disconnect()
    """

    """ LIST OF GCODE CMDS to test:
        G0 AX BX...: Move Uninterpolated

        M114: Get current Position
        M117: Get Zero Position
        M119: Get Endstop Status
        ?: Getting Information. Unclear of use, but it is called in asgard
        G28: Go to Origin (Home)
        M0: Stop 
        M18: Disable all stepper motors
        $H: homing?
        $X: Kill Alarm Command?
    """
    # Should add in some checking to ensure if/when read values differ from expected (not received or garbage), we should disconnect. 
    def __init__(self, ):
        print("HwManipulator")
        self.connection = serial.Serial(baudrate=115200)
        self.connected = False

    def connect(self, **kwargs) -> int:
        self.connection.port = kwargs["port"]
        self.connection.open()
        self.connected = True
        return 0

    def bringup(self, **kwargs) -> int:
        port = self.get_serial_port()
        if port == []:
            print("Unable to locate open serial port.")
            return -1
        elif len(port > 1):
            print(f"WARNING: More than one potential serial port matches for HW Manipulator.\n",
                    f"\t Trying with '{port[0]}'")
        try:
            self.connect(kwargs={"port": port[0]})
        except Exception as e:
            print(f"Exception Thrown: {e}")
            return -1
        return 0

    def disconnect(self, **kwargs) -> int:
        if self.connected:
            self.connection.close()
            self.connected = False
        return 0

    def move_js(self, q_array: np.ndarray):
        qs = joint_array_sanitizer(q_array)
        # Rapid uninterpolated movement        B and C must be equivalent...
        message = ( f"G0 A{str(qs[0])} B{str(qs[1])} C{str(qs[1])} D{str(qs[2])}"
                      f" X{str(qs[3])} Y{str(qs[4])} Z{str(qs[5])}\n")
        
        self.connection.write(message.encode('UTF-8'))
        print(self.connection.readline())
        # if message is "OK" then we know it worked

    def move_ts(self, pose:Pose):
        ikpy_joints = self.IK_Solver(pose)
        self.move_js(ikpy_joints)
    
    def get_joint_values(self) -> np.ndarray:
        """ Use Gcode M114 to get a string w/ current positions. 
            May also try the "?" Command as this is what Asgard used
        """
        self.connection.write("?\n".encode('UTF-8'))
        dataRead = str(self.connection.readline())
        raw_data = dataRead[1:][:-1].split(",")
        qs = np.array([[float(raw_data[1][5:][:-2])], [float(raw_data[2][:-2])], [float(raw_data[4][:-2])]
                       [float(raw_data[5][:-2])],     [float(raw_data[6][:-2])], [float(raw_data[7][:-2])]])
        qs *= (np.pi/180.0)
        return qs
    
    def get_serial_port(self) -> str:
        """ Lists serial port names. Method taken from Asgard1.0 serial_port_finder.py 
            https://github.com/AngelLM/Asgard

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result