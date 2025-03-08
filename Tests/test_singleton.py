import sys
sys.path.append('../')
from stack_a_bot.Components.Singleton_Registry import get_singleton, update_singleton_registry

from stack_a_bot.Components.Camera import Camera
from stack_a_bot.Components.HWCamera import HWCamera

from stack_a_bot.Components.Manipulator import Manipulator
from stack_a_bot.Components.SIMManipulator import SIMManipulator

# Usage
a1 = get_singleton(HWCamera)
a2 = HWCamera()

assert(a1 is a2)

update_singleton_registry(Manipulator, get_singleton(SIMManipulator))
update_singleton_registry(Camera, get_singleton(HWCamera))

a1 = get_singleton(Camera)
a2 = get_singleton(HWCamera)
b1 = get_singleton(Manipulator)
b2 = get_singleton(SIMManipulator)

assert(a1 is a2)
assert(not a1 is b1)
assert(b1 is b2)