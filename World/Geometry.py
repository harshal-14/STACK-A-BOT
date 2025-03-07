import numpy as np

## TODO: Add type checking to these classes and retest.

class RotMatrix():
    def __init__(self, rot):
        self.rot = rot
        pass

class Point():
    def __init__(self, translation):
            self.x = float(translation[0])
            self.y = float(translation[1])
            self.z = float(translation[2])

    def to_trans(self):
        return np.array([[self.x],[self.y],[self.z]])
    
class Pose():
    """A position in 3D space with orientation."""
    def __init__(self, rot, trans):
        """ Pose Constructor 
        
            Args:
                rot (RotMatrix || np.ndarray): Orientation of Axes
                trans (Point || np.ndarray): Point in space w.r.t base frame of robot. 
        """
        if type(rot) == RotMatrix:
            self.rot = rot.rot
        else:
            self.rot = rot
        if type(trans) == Point:
            self.trans = trans.to_trans()
        else:
            self.trans = trans

    def get_transform(self):
        return np.vstack((np.hstack((self.rot, self.trans)), np.array([0,0,0,1])))