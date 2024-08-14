from gello.cameras.camera import CameraDriver
from typing import Optional, Tuple
import numpy as np
from airo_camera_toolkit.cameras.zed.zed2i import Zed2i


class Zed2iCamera(CameraDriver):
    def __repr__(self) -> str:
        return f"ZedCamera(device_id={self._device_id})"

    def __init__(self, serial_number: Optional[str] = None):
        self.camera = Zed2i(resolution=Zed2i.RESOLUTION_720, fps=15, depth_mode=Zed2i.NEURAL_DEPTH_MODE, serial_number=serial_number)
        self._device_id = serial_number



    def read(
        self,
        img_size: Optional[Tuple[int, int]] = None,  # farthest: float = 0.12
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Read a frame from the camera.

        Args:
            img_size: The size of the image to return. If None, the original size is returned.
            farthest: The farthest distance to map to 255.

        Returns:
            np.ndarray: The color image, shape=(H, W, 3)
            np.ndarray: The depth image, shape=(H, W, 1)
        """
        import cv2
        color_image = self.camera.get_rgb_image_as_int()
        depth_image = self.camera._retrieve_depth_image()[..., 0]

        if img_size is None:
            image = color_image
            depth = depth_image
        else:
            image = cv2.resize(color_image, img_size)
            depth = cv2.resize(depth_image, img_size)

        depth = depth[:, :, None]

        return image, depth



if __name__ == "__main__":
    import cv2

    camera = Zed2iCamera() 

    while True:
        image_rgb, depth = camera.read()
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        cv2.imshow("image", image_bgr)
        cv2.imshow("depth", depth)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

