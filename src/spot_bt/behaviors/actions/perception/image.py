from __future__ import annotations

import os

import cv2

from bosdyn.api import image_pb2
from bosdyn.client.image import build_image_request
from bosdyn.client.image import ImageClient

import py_trees

import numpy as np

from scipy import ndimage

from spot_bt.data import Blackboards
from spot_bt.cameras import get_encoding_for_pixel_format_string
from spot_bt.cameras import IMAGE_ROTATION_ANGLES
from spot_bt.cameras import IMAGE_CAMERA_OPTIONS
from spot_bt.cameras import IMAGE_CAMERA_OPTIONS_WITH_ARM
from spot_bt.cameras import pixel_format_string_to_enum


class SwitchCamera(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, use_arm: bool = False):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.camera = None
        if use_arm:
            self.image_sources = IMAGE_CAMERA_OPTIONS_WITH_ARM
        else:
            self.image_sources = IMAGE_CAMERA_OPTIONS

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [SwitchCamera::initialise()]")
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="camera", access=py_trees.common.Access.WRITE
        )
        self.camera = self.blackboard.perception.camera

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [SwitchCamera::update()]")
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
        except:
            return py_trees.common.Status.FAILURE


    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [TakeImage::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class TakeImage(py_trees.behaviour.Behaviour):
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

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TakeImage::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="camera", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception.register_key(
            key="images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(ImageClient.default_service_name)
        if self.image_source is None:
            self.image_source = self.blackboard.perception.camera

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [TakeImage::update()]")
        try:
            image_request = [
                build_image_request(self.image_source, pixel_format=self.pixel_format)
            ]
            image_response = self.client.get_image(image_request)[0]
            dtype, n_bytes, extension = self.encoding_data

            img = np.frombuffer(image_response.shot.image.data, dtype=dtype)
            if image_response.shot.image.format == image_pb2.Image.FORMAT_RAW:
                try:
                    # Attempt to reshape array into a RGB rows X cols shape.
                    img = img.reshape(
                        (
                            image_response.shot.image.rows,
                            image_response.shot.image.cols,
                            n_bytes,
                        )
                    )
                except ValueError:
                    # Unable to reshape the image data, trying a regular decode.
                    img = cv2.imdecode(img, -1)

            else:
                img = cv2.imdecode(img, -1)

            # Autorotate image for human viewing
            if self.image_source in IMAGE_ROTATION_ANGLES:
                img = ndimage.rotate(img, IMAGE_ROTATION_ANGLES[self.image_source])

            # Save image
            self.image = img
            cv2.imwrite(f"{os.getcwd()}/test_{self.image_source}{extension}", img)

        except:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.images = self.image
        self.logger.debug(
            f" {self.name} [TakeImage::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class TakeImageAllCameras(py_trees.behaviour.Behaviour):
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
            self.image_sources = IMAGE_CAMERA_OPTIONS_WITH_ARM
        else:
            self.image_sources = IMAGE_CAMERA_OPTIONS

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TakeImageAllCameras::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(ImageClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the TakeImageAllCameras behavior when ticked."""
        self.logger.debug(f"  {self.name} [TakeImageAllCameras::update()]")
        try:
            self.images = []
            image_request = []
            for source in self.image_sources:
                image_request.append(
                    build_image_request(source, pixel_format=self.pixel_format)
                )
            image_response = self.client.get_image(image_request)
            dtype, n_bytes, extension = self.encoding_data

            for image, source in zip(image_response, self.image_sources):
                img = np.frombuffer(image.shot.image.data, dtype=dtype)
                if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                    try:
                        # Attempt to reshape array into a RGB rows X cols shape.
                        img = img.reshape(
                            (
                                image.shot.image.rows,
                                image.shot.image.cols,
                                n_bytes,
                            )
                        )
                    except ValueError:
                        # Unable to reshape the image data, trying a regular decode.
                        img = cv2.imdecode(img, -1)

                else:
                    img = cv2.imdecode(img, -1)

                # Autorotate image for human viewing
                if source in IMAGE_ROTATION_ANGLES:
                    img = ndimage.rotate(img, IMAGE_ROTATION_ANGLES[source])

                # Save image
                self.images.append(img)
                cv2.imwrite(f"{os.getcwd()}/test_{source}{extension}", img)

        except:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.images = self.images
        self.logger.debug(
            f" {self.name} [TakeImageAllCameras::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
