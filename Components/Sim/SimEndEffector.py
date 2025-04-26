from ..EndEffector import EndEffector
import pybullet as p

class SimEndEffector(EndEffector):
    """Low-level simulated End Effector.

    Because there is no vacuum gripper in PyBullet, 
    we simulate suction behavior by manually creating or removing constraints between 
    the gripper link and nearby objects.

    Attributes:
        status (int): 1 if suction is ON and object is attached, 0 otherwise.
        constraint_id (int): ID of the active PyBullet constraint, or None if no object is attached.
        robot_id (int): The PyBullet ID of the robot.
        gripper_link_index (int): The link index of the gripper in the robot URDF.
    """

    def __init__(self):
        self.status = 0
        self.constraint_id = None
        self.robot_id = None
        self.gripper_link_index = None

    def bringup(self, **kwargs) -> int:
        """ Called during system bringup. Grabs robot information. """
        self.robot_id = kwargs.get('robot_id')
        self.gripper_link_index = kwargs.get('gripper_link_index')
        return self.connect()

    def connect(self, **kwargs) -> int:
        """ Connect the simulated end effector. No real connection needed. """
        return 0

    def disconnect(self, **kwargs) -> int:
        """ Disconnects the simulated end effector by turning suction OFF. """
        self.set_mode(0)
        return 0

    def get_status(self) -> int:
        """Returns the current suction status: 1 if active, 0 if not."""
        return self.status

    def set_mode(self, on: int):
        """Turn suction ON (1) or OFF (0).
        
        Args:
            on (int): 1 to activate suction (grasp nearby object), 0 to deactivate suction (release object).
        """
        if on and self.status == 0:
            # Try to grab an object
            obj_id = self._find_nearest_object()
            if obj_id is not None:
                self._create_constraint(obj_id)
                self.status = 1
            else:
                print("[SimEndEffector] Warning: No object found to grasp.")
        
        elif not on and self.status == 1:
            # Release the currently grasped object
            self._remove_constraint()
            self.status = 0

    def _find_nearest_object(self):
        """Finds the nearest object within a small tolerance (2 cm) to the gripper."""
        tolerance = 0.02  # 2 cm
        gripper_pos, _ = p.getLinkState(self.robot_id, self.gripper_link_index)[:2]

        nearby_objects = []
        for obj_id in range(p.getNumBodies()):
            if obj_id == self.robot_id:
                continue  # Don't grab the robot itself

            obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
            distance = ((gripper_pos[0] - obj_pos[0]) ** 2 +
                        (gripper_pos[1] - obj_pos[1]) ** 2 +
                        (gripper_pos[2] - obj_pos[2]) ** 2) ** 0.5
            if distance <= tolerance:
                nearby_objects.append((distance, obj_id))

        if nearby_objects:
            nearby_objects.sort()
            return nearby_objects[0][1]
        else:
            return None

    def _create_constraint(self, obj_id: int):
        """Creates a fixed constraint between gripper and the object."""
        gripper_pos, gripper_ori = p.getLinkState(self.robot_id, self.gripper_link_index)[:2]
        obj_pos, obj_ori = p.getBasePositionAndOrientation(obj_id)

        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.gripper_link_index,
            childBodyUniqueId=obj_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[obj_pos[0] - gripper_pos[0],
                                obj_pos[1] - gripper_pos[1],
                                obj_pos[2] - gripper_pos[2]],
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=obj_ori
        )

    def _remove_constraint(self):
        """Removes the active constraint, releasing the object."""
        if self.constraint_id is not None:
            p.removeConstraint(self.constraint_id)
            self.constraint_id = None
