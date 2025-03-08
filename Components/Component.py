from abc import ABC, abstractmethod
from .Singletonmeta import SingletonMeta

class Component(ABC, metaclass=SingletonMeta):
    """ A generic non-interface specific entity that exists in the program. 
        A standardized set of routines need to be defined in the concrete impl of a comp.
        Specifically, before using a component for the first time, its bringup() routine will be called, 
        and any subsequent times the component will be attatched or detachted, calls to connect() and disconnect() will be used.
    """

    @abstractmethod
    def bringup(self, *args) -> int:
        """Completes any implementation specifc setup neccesary before component becomes operation-ready. 
            Example uses may be: 
            * Storing values read from a config file 
            * Connecting to a component via serial comm.
            * Invoking a set of test actuations on each motor.
            * Calling of simulation specific instantiations or steps
            Args:
                args (Any): Implementation specific parameters neccesary for instantiation. 
            Returns:
                status (int): Returning 0 indicates a success, and any non-zero value indicates a failure. 
        """
        raise NotImplementedError("bringup() not implemented by subclass")

    @abstractmethod
    def connect(self, *args) -> int:
        """ Establishes connection with component. Function varies by platform (Sim/Hardware) and component type (Camera, Arduino, Sensor, ...) 
            Function is blocking, implementation should specific a timeout parameter to ensure it does not hang during connection sequence.
            Args:
                args (Any): Implementation specific parameters neccesary for connection.
            Returns:
                status (int): Returning 0 indicates a success, and any non-zero value indicates a failure. 
        """
        raise NotImplementedError("connect() not implemented by subclass")

    def disconnet(self, *args) -> int:
        """ Disconnects gracefully with component. Function varies by platform (Sim/Hardware) and component type (Camera, Arduino, Sensor, ...) 
            Function is blocking, implementation should specific a timeout parameter to ensure it does not hang.
            Example uses may be: 
            Args:
                args (Any): Implementation specific parameters
            Returns:
                status (int): Returning 0 indicates a success, and any non-zero value indicates a failure. 
        """
        raise NotImplementedError("disconnect() not implemented by subclass")