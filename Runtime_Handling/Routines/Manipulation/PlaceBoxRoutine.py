from ...Status import Status, Condition
from ..Routine import Routine
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator
from ....Components.EndEffector import EndEffector
import time

class PlaceBoxRoutine(Routine):
    def __init__(self, prev_outputs: dict, parameters: dict = None):
        self.drop_pose = None
        self.predrop_pose = None
        self.manipulator: Manipulator = None
        self.end_effector: EndEffector = None

        self.status = Status(Condition.In_Progress)
        self.step = 1
        self.wait_start_time = None
        self.timeout = 5.0  # seconds timeout for movement
        self.step_start_time = None

    def init(self, prev_outputs: dict, parameters: dict = None) -> Status:
        self.drop_pose = parameters['drop_pose']
        self.predrop_pose = parameters.get('predrop_pose', self.drop_pose)
        self.manipulator: Manipulator = get_singleton(Manipulator)
        self.end_effector: EndEffector = get_singleton(EndEffector)
        self.step_start_time = time.time()
        return Status(Condition.Success)

    def loop(self) -> Status:
        if self.status.condition != Condition.In_Progress:
            return self.status

        current_time = time.time()

        # Step 1: Move to drop_pose
        if self.step == 1:
            self.manipulator.move_ts(self.drop_pose)
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
            if joint_vals:
                try:
                    current_pose = self.manipulator.FK_Solver(joint_vals)
                    if current_pose.dist(self.drop_pose) < 0.05:
                        self.step = 3
                        self.wait_start_time = current_time
                except Exception:
                    pass
            return Status(Condition.In_Progress)

        # Step 3: Release box and wait 2 seconds
        elif self.step == 3:
            self.end_effector.set_mode(0)
            if current_time - self.wait_start_time >= 2.0:
                self.manipulator.move_ts(self.predrop_pose)
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
            if joint_vals:
                try:
                    current_pose = self.manipulator.FK_Solver(joint_vals)
                    if current_pose.dist(self.predrop_pose) < 0.05:
                        self.status = Status(Condition.Success)
                        return self.status
                except Exception:
                    pass
            return Status(Condition.In_Progress)

        return Status(Condition.In_Progress)

    def end(self) -> tuple[Status, dict]:
        final_condition = Condition.Success if self.status.condition == Condition.Success else Condition.Fault
        return Status(final_condition), {"final_pose": self.manipulator.get_joint_values()}

    def handle_fault(self, prev_status: Status) -> tuple[Status, dict]:
        return Status(Condition.Fault), {}
