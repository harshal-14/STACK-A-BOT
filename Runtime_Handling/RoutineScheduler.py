from .Routines.Routine import Routine
from enum import Enum
from .Status import Status, Condition

class SchedulerState(Enum):
    INIT = 0
    LOOP = 1
    END = 2
    FAULT_HANDLER = 3

class RoutineScheduler():
    """RoutineScheduler handles the Routine Queue, where all jobs run in sequential order. 
        Routines run in a FIFO order unless specifically emplaced by another routine. """

    def __init__(self, initial_routines):
        self._routine_queue = []
        self._routine_queue.extend(initial_routines)
        self._scheduler_state = SchedulerState.INIT
        self._last_status = Status(Condition.Success)
        self._last_output = None
        
    def add_routine(self, routine:Routine):
        self._routine_queue.append(routine)
    
    def insert_now(self, routine:Routine | list):
        """ Queues a Routine to run immediately after currently active Routine."""
        if(type(routine) == list):
            iterator = 1
            for r in routine:
                self._routine_queue.insert(iterator, r)
                iterator+=1
        else:
            self._routine_queue.insert(1, routine)

    def has_routines(self) -> bool:
        if len(self._routine_queue):
            return True
        return False
    
    def handle_init(self):
        self._last_status = self._routine_queue[0].init(self._last_output)
        if self._last_status.cond == Condition.Success:
            self._scheduler_state = SchedulerState.LOOP

        elif self._scheduler_state.cond == Condition.Fault:
            self._scheduler_state = SchedulerState.FAULT_HANDLER

        elif self._scheduler_state.cond == Condition.In_Progress:
            raise RuntimeError("Invalid Status condition given")
        else:
            raise RuntimeError("Unknown Status condition given")
            

    def handle_loop(self):
        self._last_status = self._routine_queue[0].loop()
        if self._last_status.cond == Condition.Success:
            self._scheduler_state = SchedulerState.END
            
        elif self._last_status.cond == Condition.Fault:
            self._scheduler_state = SchedulerState.FAULT_HANDLER

        elif self._last_status.cond == Condition.In_Progress:
            pass # loop unfinished
        else:
            raise RuntimeError("Unknown Status condition given")
        
    def handle_end(self):
        self._last_status, self._last_output = self._routine_queue[0].end()
        if self._last_status.cond == Condition.Success:
            self._routine_queue.pop(0)
            self._scheduler_state = SchedulerState.INIT
        elif self._last_status.cond == Condition.Fault:
            self._scheduler_state = SchedulerState.FAULT_HANDLER

        elif self._last_status.cond == Condition.In_Progress:
            raise RuntimeError("Invalid Status condition given")
        else:
            raise RuntimeError("Unknown Status condition given")

    def handle_fault(self):
        """This function should contain logic to get us back on track... TODO: expand this section
            Possible actions to take:
                Some internal recorrective steps...
                Queue a set of other jobs to fix current situation...
                raise error in unrecoverable position...
        """
        self._last_status, self._last_output = self._routine_queue[0].handle_fault()
        raise self._last_status.err_type(self._last_status.err_msg)

    def run(self):
        """invokes non-blocking calls to routine's current state handler"""
        if not self.has_routines():
            pass
        elif self._scheduler_state == SchedulerState.INIT:
            self.handle_init()
        elif self._scheduler_state == SchedulerState.LOOP:
            self.handle_loop()
        elif self._scheduler_state == SchedulerState.END:
            self.handle_end()
        elif self._scheduler_state == SchedulerState.FAULT_HANDLER:
            self.handle_fault()
        else:
            raise RuntimeError("UNKNOWN Scheduler State Achieved")

    @classmethod
    def run_routines(cls, routines: list[Routine]) -> dict:
        """ Runs a set of routines by making a new scheduler object and running until completion.
            This method is BLOCKING, and is meant as a way of running a set of sequential, uninterruptable routines.
        """
        scheduler = RoutineScheduler(routines)
        while(scheduler.has_routines()):
            scheduler.run()
        return scheduler._last_output
