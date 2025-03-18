from ..Components.SingletonRegistry import get_singleton, update_singleton_registry

from ..Components.Component import Component
from ..Components.Camera import Camera
from ..Components.Manipulator import Manipulator

from ..Components.Hardware.HwCamera import HwCamera
from ..Components.Sim.SimManipulator import SimManipulator
from ..Components.Hardware.HwManipulator import HwManipulator

from ..World.SimEnvironment import SimEnvironment

from ..World.Geometry import Pose

# Ensure that singleton behavior is kept when instantiating A concrete singleton class
a2 = HwCamera()
a1 = get_singleton(HwCamera)

assert(a1 is a2)

# Ensure that calls to uninstantiated objects raise a KeyError

try:
    get_singleton(SimManipulator)
    assert(False)
except KeyError as e:
    pass

# Ensures that when we call get_singleton on a Abstract Superclass, 
# we get the concrete subclass impl

SimManipulator()
update_singleton_registry(Manipulator, get_singleton(SimManipulator))
update_singleton_registry(Camera, get_singleton(HwCamera))

a1 = get_singleton(Camera)
a2 = get_singleton(HwCamera)
b1 = get_singleton(Manipulator)
b2 = get_singleton(SimManipulator)

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

Component.lock()
try: 
    Camera()
    assert(False)
except RuntimeError as e:
    pass

# Other uninstantiated classes should also be unable to make new instances
try: 
    HwCamera()
    assert(False)
except RuntimeError as e:
    pass

Component.unlock()

# Testing SimEnvironment singleton behavior

s1 = SimEnvironment(None)
s2 = SimEnvironment(None)
s3 = get_singleton(SimEnvironment)

assert(s1 is s2)
assert(s1 is s3)

SimEnvironment.lock()
try: 
    SimEnvironment(None)
    assert(False)
except RuntimeError as e:
    pass