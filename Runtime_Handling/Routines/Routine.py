from abc import ABC, abstractmethod
from ..Status import Status

class Routine(ABC):
    """Interface providing a standardized form factor for all user defined actions of the system. 
        Routines should be designed to be queued sequentially, and may be called within other Routines.
    """
    ## NOTE: Currently unimplemented behavior -> handling sub-routines. 
    # It wasnt very clear how best to do this, so perhaps someone should undertake this endeavor... 
    
    ## NOTE: Not actually sure how we could pass in aditional parameters ATM. might be a useless field...
    @abstractmethod
    def init(self, prev_outputs:dict, parameters:dict=None) -> Status:
        """Called first in Routine Handling and runs once. Useful for setting parameters at run-time that may differ from those at instantiation
            Args:
                prev_outputs (dict): arguements from previously run Routine. 
                parameters (dict || None): *Optional*. A dictionary containing other parameters passed in at run-time
            Returns:
                status (Status): result of running init(). Status condition field determines next run function.
            """
        raise NotImplementedError("init() not implemented by subclass")

    @abstractmethod
    def loop(self) -> Status:
        """Called continuously after init returns.
            This should encompass the core functionality of the routine.
            Additionally, it must be a NON-BLOCKING FUNCTION.
            loop() stops executing once it returns Success status condition
            Returns:
                status (Status): result of running loop(). Success means end() will run next, 
                In_Progress means loop will continue to run, Fault incurs a call to handle_fault() 
        """
        raise NotImplementedError("loop() not implemented by subclass")

    @abstractmethod
    def end(self) -> tuple[Status, dict]:
        """Called once after loop returns. This function should check report the status of whatever operation was attempted during loop(), 
            and bundle information into an output dictionary for use in other Routines.    
            Returns:
                status (Status): result of running end(). 
                outputs (dict): user defined outputs from this Routine. This will be fed into next routines input function. 
                E.G final end_effector pose, bounding boxes, height map... 
        """
        raise NotImplementedError("end() not implemented by subclass")

    @abstractmethod
    def handle_fault(self, prev_status:Status) -> tuple[Status, dict]:
        """Called in the event of a reported fault status by any Routine functions. 
            This function should dictate future Actions/Routines to correct the fault.
            Args:
                prev_status (Status): The status from the last ran function of the routine. 
            Returns:
                status (Status): results of running fault_hanlder(). If condition is reported as Fault, program execution will halt.
                outputs (dict): user defined outputs for fault correction. This will be fed into next routines input function.   
            """
        raise NotImplementedError("handle_fault() not implemented by subclass")
