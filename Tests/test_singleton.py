import sys
sys.path.append('../')
from stack_a_bot.Components.SingletonRegistry import get_singleton, update_singleton_registry

from stack_a_bot.Components.Camera import Camera
from stack_a_bot.Components.HWCamera import HWCamera

from stack_a_bot.Components.Manipulator import Manipulator
from stack_a_bot.Components.SIMManipulator import SIMManipulator

from stack_a_bot.World.Geometry import Pose

# Ensure that singleton behavior is kept when instantiating A concrete singleton class
a2 = HWCamera()
a1 = get_singleton(HWCamera)

assert(a1 is a2)

# Ensure that calls to uninstantiated objects raise a KeyError

try:
    get_singleton(SIMManipulator)
    assert(False)
except KeyError as e:
    pass

# Ensures that when we call get_singleton on a Abstract Superclass, 
# we get the concrete subclass impl

SIMManipulator()
update_singleton_registry(Manipulator, get_singleton(SIMManipulator))
update_singleton_registry(Camera, get_singleton(HWCamera))

a1 = get_singleton(Camera)
a2 = get_singleton(HWCamera)
b1 = get_singleton(Manipulator)
b2 = get_singleton(SIMManipulator)

assert(a1 is a2)
assert(not a1 is b1)
assert(b1 is b2)

# Ensure none component classes can still make objects that arent singletons. (Sanity Check)
p1 = Pose(None, None)
p2 = Pose(None, None)

assert(p1 is not p2)

# Raise error when adding singletons to the registry that ARENT a Component type

try:
    get_singleton(Pose)
    assert(False) # above line should fail
except TypeError as e:
    pass


## Ensure no more instances can be called accidentially after lock is set

Camera.lock()
Camera()