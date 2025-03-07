from Routines.Routine import Routine

class RoutineScheduler():
    """RoutineScheduler handles the Routine Queue, where all jobs run in sequential order. 
        Routines run in a FIFO order unless specifically emplaced by another routine. """

    def __init__(self, initial_routines):
        self.routine_queue = []
        self.routine_queue.extend(initial_routines)
        
    def add_routine(self, routine:Routine):
        self.routine_queue.append(routine)

    def insert_now(self, routine:Routine | list):
        """ Queues a Routine to run immediately after currently active Routine."""
        if(type(routine) == list):
            iterator = 1
            for r in routine:
                self.routine_queue.insert(iterator, r)
                iterator+=1
        else:
            self.routine_queue.insert(1, routine)

    def run(self):
        # TODO: implement. Important func.
        pass

    def has_routines(self) -> bool:
        if len(self.routine_queue):
            return True
        return False
        