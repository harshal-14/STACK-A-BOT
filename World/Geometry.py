""" Objects conveying physical quantities about our system.
 
    Three main classes are defined:
        RotMatrix: Rotation Matrix in S0(3)
        Point: Metric distance in R3
        Pose: Combined object containing 6-DoF Transformation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from ..Utilities import type_checker, size_checker


class RotMatrix():
    """ S0(3) Rotation Matrix. 
        Contains methods for translating to various other representations such as 
            * (to) Euler, (to/from) Quaternion
        Attributes:
            rot (np.ndarray): Rotation Matrix (3,3)
    """

    def __init__(self, rot: np.ndarray):
        type_checker([rot], [[np.ndarray]])
        size_checker([rot], [[(1,3,3), (3,3)]])
        if rot.shape == (1,3,3):
            rot = rot.squeeze()
        self.rot = rot
    
    def to_np(self) -> np.ndarray:
        return self.rot
    
    def to_euler(self) -> np.ndarray:
        return R.from_matrix(self.rot).as_euler('xyz')
    
    """Quaternions will always be in the form [W X Y Z]"""
    def to_quat(self) -> np.ndarray:
        return R.from_matrix(self.rot).as_quat(scalar_first=True)
    
    @classmethod
    def from_quat(cls, quat: np.ndarray) -> "RotMatrix":
        return cls(R.from_quat(quat, scalar_first=True).as_matrix())

class Point():
    def __init__(self, translation):
            self.x = float(translation[0])
            self.y = float(translation[1])
            self.z = float(translation[2])

    def to_np(self) -> np.ndarray:
        return np.array([[self.x],[self.y],[self.z]])

class Pose():
    """A position in 3D space with orientation.
        Attributes:
            orient (RotMatrix): orientation of axes
            trans (Point): translation in R^3
    """

    def __init__(self, orient, trans):
        """ Pose Constructor with parameter type translation.
            Args:
                orient (RotMatrix || np.ndarray): Orientation of Axes
                trans (Point || np.ndarray): Point in space w.r.t base frame of robot. 
        """
        self.orientation = orient if type(orient) == RotMatrix else RotMatrix(orient)
        self.point = trans if type(trans) == Point else Point(trans)

    def get_transform(self):
        return np.vstack((np.hstack((self.orientation.to_np(), self.point.to_np())), np.array([0,0,0,1])))
    
    def dist(self, other_pose): 
        return np.linalg.norm(self.point.to_np(), other_pose)

    @classmethod
    def from_t(cls, t_matrix: np.ndarray) -> "Pose":
        """ Translates an R^3 transformation matrix into a Pose object.
            Args:
                t_matrix (np.ndarray): Transformation matrix of shape (4,4)"""
        type_checker([t_matrix],[[np.ndarray]])
        size_checker([t_matrix], [[(4,4)]])
        return cls(t_matrix[0:3, 0:3], t_matrix[0:3, 3])
