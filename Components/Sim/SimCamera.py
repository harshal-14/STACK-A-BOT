from ..Camera import Camera 
import numpy as np
import pybullet as p

class SimCamera(Camera):
    """Low-level integration/spoofing of simulated camera. 

    All Images/data is retrieved from pybullet replica and made available through forward-facing API calls.
    There are native ways to spoof camera data, OR, outputs can be faked all together...\n
    A good examplpe of using simulated images can be found here https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getCameraImageTest.py
    SimCamera is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Camera)`. 
        
    Attributes:
        a (type): example attribute
        b (type): example attribute
    """

    # TODO: Implement camera integration.
    def __init__(self):
        print("SimCamera")
        pass

    def bringup(self, **kwargs) -> int:
        return self.connect(kwargs=kwargs)

    def connect(self, **kwargs) -> int:
        return 0

    def disconnect(self, **kwargs) -> int:
        return 0

    def get_RGB_image(self) -> np.ndarray:
        pass

    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        pass

    def get_depth_data(self):
        pass