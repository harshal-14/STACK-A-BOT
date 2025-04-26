from ...Status import Status, Condition
from ..Routine import Routine
from ....Components.SingletonRegistry import get_singleton
from ....Components.Manipulator import Manipulator
from ....Components.EndEffector import EndEffector
import time

class GrabBoxRoutine(Routine):
    def __init__(self, prev_outputs: dict, parameters: dict = None):
        self.pickup_pose = None
        self.prepickup_pose = None
        self.manipulator: Manipulator = None
        self.end_effector: EndEffector = None

        self.status = Status(Condition.In_Progress)
        self.step = 1
        self.wait_start_time = None
        self.timeout = 5.0  # seconds timeout for movement
        self.step_start_time = None

    def init(self, prev_outputs: dict, parameters: dict = None) -> Status:
        self.pickup_pose = parameters['pickup_pose']
        self.prepickup_pose = parameters.get('prepickup_pose', self.pickup_pose)
        self.manipulator: Manipulator = get_singleton(Manipulator)
        self.end_effector: EndEffector = get_singleton(EndEffector)
        self.step_start_time = time.time()
        return Status(Condition.Success)

    def loop(self) -> Status:
        if self.status.condition != Condition.In_Progress:
            return self.status

        current_time = time.time()

        # Step 1: Enable suction and move to pickup_pose
        if self.step == 1:
            self.end_effector.set_mode(1)
            self.manipulator.move_ts(self.pickup_pose)
            self.step = 2
            self.step_start_time = current_time
            return Status(Condition.In_Progress)

        # Step 2: Wait to reach pickup_pose or timeout
        elif self.step == 2:
            if current_time - self.step_start_time > self.timeout:
                print("[GrabBoxRoutine] Timeout reaching pickup_pose")
                self.status = Status(Condition.Fault)
                return self.status

            joint_vals = self.manipulator.get_joint_values()
            if joint_vals:
                try:
                    current_pose = self.manipulator.FK_Solver(joint_vals)
                    if current_pose.dist(self.pickup_pose) < 0.05:  # Loose threshold
                        self.step = 3
                        self.wait_start_time = current_time
                except Exception:
                    pass
            return Status(Condition.In_Progress)

        # Step 3: Wait 2 seconds at pickup_pose
        elif self.step == 3:
            if current_time - self.wait_start_time >= 2.0:
                self.manipulator.move_ts(self.prepickup_pose)
                self.step = 4
                self.step_start_time = current_time
            return Status(Condition.In_Progress)

        # Step 4: Wait to reach prepickup_pose or timeout
        elif self.step == 4:
            if current_time - self.step_start_time > self.timeout:
                print("[GrabBoxRoutine] Timeout reaching prepickup_pose")
                self.status = Status(Condition.Fault)
                return self.status

            joint_vals = self.manipulator.get_joint_values()
            if joint_vals:
                try:
                    current_pose = self.manipulator.FK_Solver(joint_vals)
                    if current_pose.dist(self.prepickup_pose) < 0.05:
                        self.status = Status(Condition.Success)
                        return self.status
                except Exception:
                    pass
            return Status(Condition.In_Progress)

        return Status(Condition.In_Progress)

    def end(self) -> tuple[Status, dict]:
        # Return Success or Fault depending on final status
        final_condition = Condition.Success if self.status.condition == Condition.Success else Condition.Fault
        return Status(final_condition), {"final_pose": self.manipulator.get_joint_values()}

    def handle_fault(self, prev_status: Status) -> tuple[Status, dict]:
        return Status(Condition.Fault), {}
