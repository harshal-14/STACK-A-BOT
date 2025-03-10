from ..Routine import Routine
from .. .Status import Status, Condition
class ComponentShutdown(Routine):
    # TODO implement :)
    def __init__(self):
        super().__init__()
        pass

    def init(self, prev_outputs, parameters = None) -> Status:
        return super().init(prev_outputs, parameters)
    
    def loop(self) -> Status:
        return super().loop()
    
    def end(self):
        return super().end()
    
    def handle_fault(self):
        return super().handle_fault()