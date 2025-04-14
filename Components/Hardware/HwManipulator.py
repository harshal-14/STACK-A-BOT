from ..Manipulator import Manipulator 
import numpy as np
from ...World.Geometry import Pose, Point 
from ...Utilities import joint_array_sanitizer

import glob
import serial
import sys
import threading
import time

class HwManipulator(Manipulator):
    """Low-level integration & communication with Manipulator.

    HwManipulator is an arduino connected via serial. High level requests must be translated into GCode and sent via serial to the arduino. 
    For more info, look at http://thor.angel-lm.com/documentation/firmware/. and https://reprap.org/wiki/G-code\n
    HwManipulator is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Manipulator)`. 
    
    Attributes:
        connection (serial.Serial): Serial connection to Manipulator Arduino
        connected (bool): Status of Connection. Coupled only via calles to connect() and disconnect()
    """

    """ All of the following work:
        G0 AX BX...: Move Uninterpolated
        ?: Getting Information
        !: feed hold - might want to use M1 instead
        ~: resumes control, 
        G90: Set to Absolute Positioning 
        G91: Set to Relative Positioning
        M1: Stop  - alternative to ! (probably works)
        $X: Kill Alarm Command?
        G28: Go to Origin (Home)
        """
    def __init__(self, ):
        print("HwManipulator")
        self.connection = serial.Serial(baudrate=115200)
        self.connected = False
        self.current_position = None
        # Threading objects
        self.position_thread = threading.Thread(None, self.update_joint_values, "Position thread")
        self._pos_lock = threading.Lock()
        self._rw_lock = threading.Lock()

    def __del__(self):
        self.stop()

    def stop(self):
        if self.connected:
            self.connection.write("!".encode("UTF-8"))
    
    def connect(self, **kwargs) -> int:
        self.connection.port = kwargs["port"]
        self.connection.open()
        # make two calls to read, one to get first empty msg, and the second to get the intro sequence
        dataRead = str(self.connection.readline().decode("utf-8")).strip("\r\n")
        dataRead = str(self.connection.readline().decode("utf-8")).strip("\r\n")
        print(dataRead)
        self.connected = True
        self.position_thread.start()
        return 0

    def bringup(self, **kwargs) -> int:
        port = self.get_serial_port()
        if port == []:
            print("Unable to locate open serial port.")
            return -1
        elif len(port) > 1:
            print(f"WARNING: More than one potential serial port matches for HW Manipulator.\n",
                    f"\t Trying with '{port[0]}'")
        try:
            self.connect(port=port[0])
        except Exception as e:
            print(f"Exception Thrown: {e}")
            return -1
        
        # wait until we get a successful readout of our current position
        position_iter_attempts = 0
        while True:
            with self._pos_lock:
                if(self.current_position is not None): break
            position_iter_attempts+=1
            if(position_iter_attempts > 5):
                print("Unable to read position from Manipulator")
                return -1
            time.sleep(0.5)
        return 0

    def disconnect(self, **kwargs) -> int:
        if self.connected:
            self.connected = False
            try:
                self.position_thread.join(timeout=2)
            except RuntimeError as e:
                print(e)
                return -1
            self.connection.close()
        return 0

    def move_js(self, q_array: np.ndarray):
        """GRBL reccomends different strategies for sending GCode to the machine [seen here](https://github.com/grbl/grbl/wiki/Interfacing-with-Grbl).\n 
            First method - Simple Send-Response. 
            \nSend command and wait for ok response before sending another one
                \n\tPros: 
                     * Ensures sent package wasn't garbled by communication interface. 
                     * Ensures that sent command is parsable and no errors are thrown for a faulty command.
                     * Running other methods won't accidentially receive wrong response msg?
                \n\tCons: 
                     * Slow and potentially jerky motion when running with small differences in joint values

            Second method - Streming via Flow Control
            \nAllows commands to be sent uninterrupted until a special XON/XOFF signal has been sent. Dependant on the state of the receiving buffer
                \n\tPros: 
                     * Takes full advantage of the look-ahead buffer for smooth motion.
                     * Potentially better for our non-blocking architecture assuming we aren't bottlenecked by loop frequency.
                \n\tCons: 
                     * Broken depending on specific Arduino USB-serial converter chip. (Need to verify)
                     * Not as robust as method 1. May need system specific tuning of high-low watermarks. 
                     * Not officially reccomended by GRBL team. 
                     * May need its own thread / logic to prevent other methods from accidentially receiving XON/XOFF characters. 
        
            Third Method - Character-Counting.
            \nHost keeps a running total of characters sent via serial and only decides to send another message once there is enough room in the buffer.
            \nWhen grbl responds, we know that a command has been processed and the data has been removed from the rx-buffer.
                \n\tPros: 
                     * Safe like Method 1 and optimal like method 2.
                     * "Not hard" to implement (see their example code in stream.py)   
                \n\tCons: 
                     * Unsure how it will interface w/ other methods. Will we need special RX/TX handler functions?
        """     

        qs = joint_array_sanitizer(q_array)
        # Rapid uninterpolated movement        B and C must be equivalent...
        message = ( f"G0 A{qs[0]:.3f} B{qs[1]:.3f} C{qs[1]:.3f} D{qs[2]:.3f}"
                      f" X{qs[3]:.3f} Y{qs[4]:.3f} Z{qs[5]:.3f}\n")
        
        with self._rw_lock:
            self.connection.write(message.encode("UTF-8"))
            retmsg = str(self.connection.readline().decode("utf-8")).strip("\r\n")
        if retmsg != "ok":
            raise RuntimeError("HWManipulator.move_js() returned something other than 'ok'.\n"
                               f"potential fault msg: {retmsg}")

    def move_ts(self, pose:Pose):
        ikpy_joints = self.IK_Solver(pose)
        self.move_js(ikpy_joints)

    def update_joint_values(self):
        """ Function to query grbl for current position, 
            \nAs reccomended by GRBL documentation, this runs at 10Hz max, any faster yields diminshing returns.
        """
        print("Starting position thread")
        while True:
            if not self.connected: break
            with self._rw_lock:
                self.connection.write("?".encode('UTF-8'))
                dataRead = str(self.connection.readline().decode("utf-8")).strip("\r\n")
                print(dataRead)
            if "error" in dataRead or "ALARM" in dataRead:
                print(f"Error encountered: {dataRead}")
            else:
                raw_data = dataRead[1:][:-1].split(",")
                qs = np.array([[float(raw_data[1][5:])], [float(raw_data[2])], [float(raw_data[4])], 
                               [float(raw_data[5])], [float(raw_data[6])], [float(raw_data[7])]])
                qs *= (np.pi/180.0)
                with self._pos_lock:
                    self.current_position = qs
            time.sleep(0.1) # sleep for 100ms 
    
    def get_joint_values(self) -> np.ndarray:
        """ Use Gcode M114 to get a string w/ current positions. 
            May also try the "?" Command as this is what Asgard used

            So Grbl reccomends only querying the position at 5Hz-10Hz maximum.
            Perhaps a thread is neccesary here to continually get the position and update its associated variable. 
        """
        retval = None
        with self._pos_lock:
            retval = self.current_position
        return retval
    
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