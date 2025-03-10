from ..Camera import Camera 
import numpy as np
class HwCamera(Camera):
    """Low-level integration & communication with OAK-D camera. 

    All Images/data is retrieved from the physical camera and made available through forward-facing API calls.\n
    HwCamera is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Camera)`. 
        
    Attributes:
        a (type): example attribute
        b (type): example attribute
    """

    # TODO: Implement camera integration.
    def __init__(self):
        print("HwCamera")
        pass

    def bringup(self, **kwargs) -> int:
        """Add additional parameters if neccesary"""
        return self.connect(kwargs=kwargs)

    def connect(self, **kwargs) -> int:
        return 0

    def disconnet(self, **kwargs) -> int:
        return 0

    def get_RGB_image(self) -> np.ndarray:
        pass

    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        pass

    def get_depth_data(self) -> ...:
        pass