from ..singleton_meta import SingletonMeta
import serial
import threading
import sys
import glob

class HwInterface(metaclass=SingletonMeta):
    """ Exposes the serial communications protocols for using grbl on an Arduino. 

    Message RX/TX should be syncronous,
    as this is the easist way to ensure messages reach their correct endpoints.
    A syncronization primative is used to ensure mutual exclusion when reading/writing.
        
    Valid GCode messages:
        G0 AX BX...: Move Uninterpolated
        ?: Getting Information
        !: feed hold - might want to use M1 instead
        ~: resumes control, 
        G90: Set to Absolute Positioning 
        G91: Set to Relative Positioning
        M1: Stop  - alternative to ! (probably works)
        M3 sX: Sets the spindle (or vacuum EE in our case) to a set speed
        $X: Kill Alarm Command?
        G28: Go to Origin (Home)
        
    Attributes:
        connection (serial.Serial): Serial connection to Manipulator Arduino
        connected (bool): Status of Connection. Coupled only via calles to connect() and disconnect()
    """

    """ Left unimplemented for the sake of time:
        
        A system should be implement such that we track the number of active components using this connection.
        Upon calls to connect and disconnect, we should increment/decrement this number.
        If we ever get back to zero, we should disconnect from serial. 
    """
    def __init__(self):
        self.connection = serial.Serial(baudrate=115200)
        self._rw_lock = threading.Lock()
        self.connected = False

    def __del__(self):
        if self.connected:
            self.connection.close()

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
    
    def connect_device(self):
        """ Connects to serial device based on available ports on device.
        
        If serial is not connected, search for available serial port, and begin to recieve starter two messages
        If already connected, method short circuits.
        """
        if self.connected: return 0
        port = self.get_serial_port()
        if port == []:
            print("Unable to locate open serial port.")
            return -1
        elif len(port) > 1:
            print(f"WARNING: More than one potential serial port matches for HW Manipulator.\n",
                    f"\t Trying with '{port[0]}'")
            
        try:
            self.connection.port = port[0]
            self.connection.open()
            # make two calls to read, one to get first empty msg, and the second to get the intro sequence
            dataRead = str(self.connection.readline().decode("utf-8")).strip("\r\n")
            dataRead = str(self.connection.readline().decode("utf-8")).strip("\r\n")
            print(dataRead)
            self.connected = True
            return 0
        except Exception as e:
            print(f"Exception Thrown: {e}")
            return -1
    
    def tx_rx(self, msg:str) -> str:
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
        with self._rw_lock:
            self.connection.write(msg.encode('UTF-8'))
            dataRead = str(self.connection.readline().decode("utf-8")).strip("\r\n")
        return dataRead
