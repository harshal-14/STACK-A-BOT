from enum import Enum

class Condition(Enum):
    Success = 0
    In_Progress = 1
    Fault = -1

class Status():
    def __init__(self, cond: Condition, err_msg:str="", fault_params:dict=None):
        self.condition = cond
        self.err_msg = err_msg
        self.fault_params = fault_params