import os
import time
from typing import List, Optional, Tuple

import numpy as np

from gello.cameras.camera import CameraDriver
import pyrealsense2 as rs


def get_device_ids() -> List[str]:
    import pyrealsense2 as rs

    ctx = rs.context()
    devices = ctx.query_devices()
    device_ids = []
    for dev in devices:
        dev.hardware_reset()
        device_ids.append(dev.get_info(rs.camera_info.serial_number))
    time.sleep(2)
    return device_ids


class RealSenseCamera(CameraDriver):
    def __repr__(self) -> str:
        return f"RealSenseCamera(device_id={self._device_id})"

    def __init__(self, device_id: Optional[str] = None, flip: bool = False):
        self.camera = Realsense(fps=30,resolution=(640,480),serial_number=device_id)
        self._flip = flip
        self._device_id = device_id

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
        depth_image = self.camera.retrieve_depth_image()[..., 0]
        color_image = self.camera.retrieve_rgb_image_as_int()
        # depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)
        if img_size is None:
            image = color_image
            depth = depth_image
        else:
            image = cv2.resize(color_image, img_size)
            depth = cv2.resize(depth_image, img_size)

        # rotate 180 degree's because everything is upside down in order to center the camera
        if self._flip:
            image = cv2.rotate(image, cv2.ROTATE_180)
            depth = cv2.rotate(depth, cv2.ROTATE_180)[:, :, None]
        else:
            depth = depth[:, :, None]

        return image, depth



class Realsense:
    """Wrapper around the pyrealsense2 library to use the RealSense cameras (tested for the D415 and D435).

    taken from https://github.com/airo-ugent/airo-mono/blob/main/airo-camera-toolkit/airo_camera_toolkit/cameras/realsense/realsense.py


    """

    # Built-in resolutions (16:9 aspect ratio) for convenience
    # for all resolutions see: realsense_scan_profiles.py
    RESOLUTION_1080 = (1920, 1080)
    RESOLUTION_720 = (1280, 720)
    RESOLUTION_540 = (960, 540)
    RESOLUTION_480 = (848, 480)

    def __init__(
        self,
        resolution = RESOLUTION_1080,
        fps: int = 30,
        enable_depth: bool = True,
        enable_hole_filling: bool = False,
        serial_number: Optional[str] = None,
    ) -> None:
        self._resolution = resolution
        self.fps = fps
        self._depth_enabled = enable_depth
        self.hole_filling_enabled = enable_hole_filling
        self.serial_number = serial_number

        config = rs.config()

        if serial_number is not None:
            # Note: Invalid serial_number leads to RuntimeError for pipeline.start(config)
            config.enable_device(serial_number)

        config.enable_stream(rs.stream.color, self._resolution[0], self._resolution[1], rs.format.rgb8, fps)

        if self._depth_enabled:
            config.enable_stream(rs.stream.depth,640,480, rs.format.z16, fps)

        # Avoid having to reconnect the USB cable, see https://github.com/IntelRealSense/librealsense/issues/6628#issuecomment-646558144
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()

        self.pipeline = rs.pipeline()

        self.pipeline.start(config)

        # Get intrinsics matrix
        profile = self.pipeline.get_active_profile()
   

        if self._depth_enabled:
            device = profile.get_device()
            depth_sensor = device.first_depth_sensor()
            self.depth_factor = depth_sensor.get_depth_scale()
            self._setup_depth_transforms()
            self.colorizer = rs.colorizer()
            self.colorizer.set_option(rs.option.color_scheme, 2)  # 2 = White to Black


    def _setup_depth_transforms(self) -> None:
        # Configure depth filters and transfrom, adapted from:
        # https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
        # https://github.com/IntelRealSense/librealsense/blob/jupyter/notebooks/depth_filters.ipynb
        self.align_transform = rs.align(rs.stream.color)
        self.hole_filling = rs.hole_filling_filter()

    def _grab_images(self) -> None:
        self._composite_frame = self.pipeline.wait_for_frames()

        if not self._depth_enabled:
            return

        aligned_frames = self.align_transform.process(self._composite_frame)
        self._depth_frame = aligned_frames.get_depth_frame()

        if self.hole_filling_enabled:
            self._depth_frame = self.hole_filling.process(self._depth_frame)


    def retrieve_rgb_image_as_int(self):
        self._grab_images()
        assert isinstance(self._composite_frame, rs.composite_frame)
        color_frame = self._composite_frame.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        return image

    def retrieve_depth_map(self):
        self._grab_images()
        frame = self._depth_frame
        image = np.asanyarray(frame.get_data()).astype(np.float32)
        return image * self.depth_factor

    def retrieve_depth_image(self):
        self._grab_images()
        frame = self._depth_frame
        frame_colorized = self.colorizer.colorize(frame)
        image = np.asanyarray(frame_colorized.get_data())  # this is uint8 with 3 channels
        return image
    
def _debug_read(camera, save_datastream=False):
    import cv2

    cv2.namedWindow("image")
    cv2.namedWindow("depth")
    counter = 0
    if not os.path.exists("images"):
        os.makedirs("images")
    if save_datastream and not os.path.exists("stream"):
        os.makedirs("stream")
    while True:
        time.sleep(0.1)
        image, depth = camera.read()
        depth = np.concatenate([depth, depth, depth], axis=-1)
        key = cv2.waitKey(1)
        cv2.imshow("image", image[:, :, ::-1])
        cv2.imshow("depth", depth)
        if key == ord("s"):
            cv2.imwrite(f"images/image_{counter}.png", image[:, :, ::-1])
            cv2.imwrite(f"images/depth_{counter}.png", depth)
        if save_datastream:
            cv2.imwrite(f"stream/image_{counter}.png", image[:, :, ::-1])
            cv2.imwrite(f"stream/depth_{counter}.png", depth)
        counter += 1
        if key == 27:
            break


if __name__ == "__main__":
    device_ids = get_device_ids()
    print(f"Found {len(device_ids)} devices")
    print(device_ids)
    camera = RealSenseCamera(device_id=device_ids[1])
    #_debug_read(camera)
    img = camera.read()
    rgb = img[0]
    from PIL import Image
    img = Image.fromarray(rgb)
    img.save("camera-pose.png")
