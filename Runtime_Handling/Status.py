from enum import Enum

class Condition(Enum):
    Success = 0
    In_Progress = 1
    Fault = -1

class Status():
    """Routine communication object. 
    
    Wrapper around enumator w/ additional error handling capabilities packed in.

    Attributes:
        cond (Condition): enumerator denoting result of running Routine method
        err_msg (str): string denoting error encountered in case of a fault
        err_type (BaseException): type of error encountered
        fault_params: dictionary of kwargs to be used in fault recovery.
    """
    def __init__(self, cond: Condition, err_msg:str="", err_type:BaseException=RuntimeError, fault_params:dict=None):
        self.cond = cond
        self.err_msg = err_msg
        self.err_type = err_type
        self.fault_params = fault_params