# from ..Camera import Camera 
# import numpy as np
# import pybullet as p

# class SimCamera(Camera):
#     """Low-level integration/spoofing of simulated camera. 

#     All Images/data is retrieved from pybullet replica and made available through forward-facing API calls.
#     There are native ways to spoof camera data, OR, outputs can be faked all together...\n
#     A good examplpe of using simulated images can be found here https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getCameraImageTest.py
#     SimCamera is a singleton object and can be refered to via `SingletonRegistry.get_singleton(Camera)`. 
        
#     Attributes:
#         a (type): example attribute
#         b (type): example attribute
#     """

#     # TODO: Implement camera integration.
#     def __init__(self):
#         print("SimCamera")
#         pass

#     def bringup(self, **kwargs) -> int:
#         return self.connect(kwargs=kwargs)

#     def connect(self, **kwargs) -> int:
#         return 0

#     def disconnect(self, **kwargs) -> int:
#         return 0

#     def get_RGB_image(self) -> np.ndarray:
#         pass

#     def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
#         pass

#     def get_depth_data(self):
#         pass
from ..Camera import Camera 
#from Camera import Camera

import numpy as np
import pybullet as p

class SimCamera(Camera):
    """Low-level integration/spoofing of simulated camera. 

    All Images/data is retrieved from pybullet replica and made available through forward-facing API calls.
    """

    def __init__(self):
        # Set up camera parameters
        self.img_width = 640
        self.img_height = 480
        self.fov = 60
        self.aspect = self.img_width / self.img_height
        self.near = 0.1
        self.far = 10.0

        # Camera position and orientation (default values; can be overwritten)
        self.camera_pos = [1, 1, 1]
        self.camera_target = [0, 0, 0]
        self.up_vector = [0, 0, 1]

    def bringup(self, **kwargs) -> int:
        return self.connect(**kwargs)

    def connect(self, **kwargs) -> int:
        # Optionally override camera settings from kwargs
        self.camera_pos = kwargs.get('camera_pos', self.camera_pos)
        self.camera_target = kwargs.get('camera_target', self.camera_target)
        return 0

    def disconnect(self, **kwargs) -> int:
        # No special teardown required for now
        return 0

    def get_RGB_image(self) -> np.ndarray:
        view_matrix = p.computeViewMatrix(self.camera_pos, self.camera_target, self.up_vector)
        projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)

        _, _, rgba_img, _, _ = p.getCameraImage(
            width=self.img_width,
            height=self.img_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        rgba_np = np.array(rgba_img, dtype=np.uint8).reshape((self.img_height, self.img_width, 4))
        rgb_np = rgba_np[:, :, :3].transpose(2, 0, 1).astype(np.float32) / 255.0  # D x H x W
        return rgb_np

    def get_mono_images(self) -> tuple[np.ndarray, np.ndarray]:
        rgb = self.get_RGB_image()  # D x H x W
        # Convert RGB to grayscale using luminance-preserving formula
        gray = 0.2989 * rgb[0] + 0.5870 * rgb[1] + 0.1140 * rgb[2]
        mono = gray[np.newaxis, :, :].astype(np.float32)  # shape: (1, H, W)
        return (mono, mono.copy())  # Simulate two mono cameras

    def get_depth_data(self) -> np.ndarray:
        view_matrix = p.computeViewMatrix(self.camera_pos, self.camera_target, self.up_vector)
        projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)

        _, _, _, depth_buffer, _ = p.getCameraImage(
            width=self.img_width,
            height=self.img_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        depth_np = np.array(depth_buffer).reshape((self.img_height, self.img_width)).astype(np.float32)
        depth_np = depth_np[np.newaxis, :, :]  # shape: (1, H, W)
        return depth_np
