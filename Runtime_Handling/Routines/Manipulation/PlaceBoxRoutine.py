from ...Status import Status, Condition
from ..Routine import Routine
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator
from ....Components.EndEffector import EndEffector
import time
import numpy as np

class PlaceBoxRoutine(Routine):
    def __init__(self, drop_pose):
        self.manipulator: Manipulator
        self.end_effector: EndEffector
        self.prepickup_pose:np.ndarray
        self.drop_pose = drop_pose

        self.step = 1
        self.wait_start_time = None
        self.step_start_time = None

    def init(self, prev_outputs: dict, parameters: dict = None) -> Status:
        self.manipulator: Manipulator = get_singleton(Manipulator)
        self.end_effector: EndEffector = get_singleton(EndEffector)
        self.step_start_time = time.time()
        self.predrop_pose = self.manipulator.get_joint_values()
        return Status(Condition.Success)

    def loop(self) -> Status:
        current_time = time.time()

        # Step 1: Move to drop_pose
        if self.step == 1:
            self.manipulator.move_js(self.drop_pose)
            self.step = 2
            self.step_start_time = current_time

        # Step 2: Wait to reach drop_pose or timeout
        elif self.step == 2:
            if current_time - self.step_start_time > 8:
                self.step = 3
                self.wait_start_time = current_time

        # Step 3: Release box and wait 3 seconds
        elif self.step == 3:
            if self.end_effector.get_status():
                self.end_effector.set_mode(False)
            if current_time - self.wait_start_time >= 5.0:
                self.manipulator.move_js(self.predrop_pose)
                self.step = 4
                self.step_start_time = current_time

        # Step 4: Wait to reach predrop_pose or timeout
        elif self.step == 4:
            if current_time - self.step_start_time > 7:
                return Status(Condition.Success)
        
        return Status(Condition.In_Progress)

    def end(self) -> tuple[Status, dict]:
        time.sleep(5)
        return Status(Condition.Success), {"final_pose": self.manipulator.get_joint_values()}

    def handle_fault(self, prev_status: Status) -> tuple[Status, dict]:
        return Status(Condition.Fault), {}
