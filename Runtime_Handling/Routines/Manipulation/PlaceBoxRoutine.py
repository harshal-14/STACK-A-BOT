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
        self.timeout = 5.0  # seconds timeout for movement
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
            return Status(Condition.In_Progress)

        # Step 2: Wait to reach drop_pose or timeout
        elif self.step == 2:
            if current_time - self.step_start_time > self.timeout:
                print("[PlaceBoxRoutine] Timeout reaching drop_pose")
                self.status = Status(Condition.Fault)
                return self.status

            joint_vals = self.manipulator.get_joint_values()
            current_pose = self.manipulator.FK_Solver(joint_vals)
            if current_pose.dist(self.drop_pose) < 3:
                self.step = 3
                self.wait_start_time = current_time
            return Status(Condition.In_Progress)

        # Step 3: Release box and wait 2 seconds
        elif self.step == 3:
            self.end_effector.set_mode(0)
            if current_time - self.wait_start_time >= 2.0:
                self.manipulator.move_js(self.predrop_pose)
                self.step = 4
                self.step_start_time = current_time
            return Status(Condition.In_Progress)

        # Step 4: Wait to reach predrop_pose or timeout
        elif self.step == 4:
            if current_time - self.step_start_time > self.timeout:
                print("[PlaceBoxRoutine] Timeout reaching predrop_pose")
                self.status = Status(Condition.Fault)
                return self.status

            joint_vals = self.manipulator.get_joint_values()
            current_pose = self.manipulator.FK_Solver(joint_vals)
            if current_pose.dist(self.predrop_pose) < 3:
                return Status(Condition.Success)
            return Status(Condition.In_Progress)

    def end(self) -> tuple[Status, dict]:
        return Status(Condition.Success), {"final_pose": self.manipulator.get_joint_values()}

    def handle_fault(self, prev_status: Status) -> tuple[Status, dict]:
        return Status(Condition.Fault), {}
