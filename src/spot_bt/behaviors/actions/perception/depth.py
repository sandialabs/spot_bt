from __future__ import annotations

from bosdyn.client.image import build_image_request, ImageClient

import py_trees

import numpy as np

from spot_bt.data import Blackboards
from spot_bt.cameras import (
    get_encoding_for_pixel_format_string,
    DEPTH_CAMERA_OPTIONS,
    DEPTH_CAMERA_OPTIONS_WITH_ARM,
    pixel_format_string_to_enum,
)


class SwitchDepthCamera(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, use_arm: bool = False):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.camera = None
        self.images = None
        if use_arm:
            self.image_sources = DEPTH_CAMERA_OPTIONS_WITH_ARM
        else:
            self.image_sources = DEPTH_CAMERA_OPTIONS

    def setup(self, **kwargs):
        """Setup SwitchDepthCamera behavior before initialization."""
        self.logger.debug(f"  {self.name} [SwitchDepthCamera::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [SwitchDepthCamera::initialise()]")
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="depth_camera", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception.register_key(
            key="images", access=py_trees.common.Access.WRITE
        )
        self.camera = self.blackboard.perception.depth_camera
        self.images = self.blackboard.perception.image

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [SwitchDepthCamera::update()]")
        try:
            if self.camera is None:
                self.camera = self.image_sources[0]
            else:
                idx = self.image_sources.index(self.camera)
                if idx == len(self.image_sources) - 1:
                    self.camera = self.image_sources[0]
                else:
                    self.camera = self.image_sources[idx + 1]

            return py_trees.common.Status.SUCCESS

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.images = self.images
        self.logger.debug(
            f" {self.name} [SwitchDepthCamera::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class TakeDepthImage(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        image_source: str | None = None,
        pixel_format: str = "PIXEL_FORMAT_RGB_U8",
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.image = None
        self.image_source = image_source
        self.pixel_format = pixel_format_string_to_enum(pixel_format)
        self.encoding_data = get_encoding_for_pixel_format_string(pixel_format)

    def setup(self, **kwargs):
        """Setup TakeDepthImage behavior before initialization."""
        self.logger.debug(f"  {self.name} [TakeDepthImage::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TakeDepthImage::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="depth_camera", access=py_trees.common.Access.READ
        )
        self.blackboard.perception.register_key(
            key="depth_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(ImageClient.default_service_name)
        if self.image_source is None:
            self.image_source = self.blackboard.perception.depth_camera

    def update(self) -> py_trees.common.Status:
        """Run the TakeDepthImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [TakeDepthImage::update()]")
        try:
            image_request = [
                build_image_request(self.image_source, pixel_format=self.pixel_format)
            ]
            image_response = self.client.get_image(image_request)[0]
            dtype, n_bytes, extension = self.encoding_data
            cv_depth = np.frombuffer(image_response.shot.image.data, dtype=np.uint16)
            cv_depth = cv_depth.reshape(
                image_response.shot.image.rows, image_response.shot.image.cols
            )

            # Save image
            self.image = cv_depth

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.images = self.image
        self.logger.debug(
            f" {self.name} [TakeDepthImage::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class TakeDepthImageAllCameras(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        pixel_format: str = "PIXEL_FORMAT_RGB_U8",
        use_arm: bool = False,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.images = None
        self.pixel_format = pixel_format_string_to_enum(pixel_format)
        self.encoding_data = get_encoding_for_pixel_format_string(pixel_format)
        if use_arm:
            self.image_sources =DEPTH_CAMERA_OPTIONS_WITH_ARM
        else:
            self.image_sources = DEPTH_CAMERA_OPTIONS

    def setup(self, **kwargs):
        """Setup TakeDepthImageAllCameras behavior before initialization."""
        self.logger.debug(f"  {self.name} [TakeDepthImageAllCameras::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TakeDepthImageAllCameras::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="depth_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(ImageClient.default_service_name)
        print("DONE INITIALIZING")

    def update(self) -> py_trees.common.Status:
        """Run the TakeImageAllCameras behavior when ticked."""
        self.logger.debug(f"  {self.name} [TakeDepthImageAllCameras::update()]")
        try:
            print("STARTING UPDATE")
            self.images = []
            # image_request = []
            # image_request.append(
            #     build_image_request(self.image_sources, pixel_format=self.pixel_format)
            # )
            print("SENDING REQUEST")
            image_response = self.client.get_image_from_sources(self.image_sources)
            # image_response = self.client.get_image(image_request)
            dtype, n_bytes, extension = self.encoding_data
            print("RECEIVED RESPONSE")
            for image, source in zip(image_response, self.image_sources):
                print(f"{source}")
                cv_depth = np.frombuffer(image.shot.image.data, dtype=np.uint16)
                cv_depth = cv_depth.reshape(
                    image.shot.image.rows, image.shot.image.cols
                )

                # Append depth image
                self.images.append(cv_depth)
                print(f"SAVED {source}")

        except:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.depth_images = self.images
        self.logger.debug(
            f" {self.name} [TakeDepthImageAllCameras::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
