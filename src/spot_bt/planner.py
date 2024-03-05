"""Autonomy planner for Spot to use."""
import numpy as np

from spot_bt.cameras import IMAGE_CAMERA_OPTIONS
from spot_bt.cameras import IMAGE_CAMERA_OPTIONS_WITH_ARM
from spot_bt.cameras import DEPTH_CAMERA_OPTIONS
from spot_bt.cameras import DEPTH_CAMERA_OPTIONS_WITH_ARM


class Planner:
    def __init__(
        self, initial_state: str = "exploration", with_arm: bool = False
    ):
        self.state = initial_state
        self.with_arm = with_arm
        if with_arm:
            self.sources = {
                "depth": DEPTH_CAMERA_OPTIONS_WITH_ARM,
                "rgb": IMAGE_CAMERA_OPTIONS_WITH_ARM,
            }
        else:
            self.sources = {
                "depth": DEPTH_CAMERA_OPTIONS,
                "rgb": IMAGE_CAMERA_OPTIONS,
            }

    def get_new_heading(
        self, depth_images: list[np.ndarray], threshold: float = 1.0
    ) -> float:
        """Get a new heading given Depth camera information."""
        averages = [np.mean(cv_depth) for cv_depth in depth_images]
        for source, average in zip(self.sources["depth"], averages):
            print(source, average)

        return 0
