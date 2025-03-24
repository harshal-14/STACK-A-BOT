import sys
sys.path.append('../')
# sys.__package__ = 

from stack_a_bot.World.Geometry import RotMatrix, Point, Pose
import numpy as np

## TODO: Add unit testing framework 

rot = RotMatrix(np.random.rand(3,3))
trans = Point(np.random.rand(3,1))

pose = Pose(rot,trans)
print(pose.orientation)
print(pose.point)
print(np.hstack((pose.orientation, pose.point)))

print(pose.get_transform())