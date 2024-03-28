"""spot_bt Selector composites."""
from __future__ import annotations

from py_trees.composites import Selector

from spot_bt.behaviors.actions.perception import DetectFiducialMarkers
from spot_bt.behaviors.conditions.perception import IsAnyFiducialMarkerDetected
from spot_bt.behaviors.conditions.general import IsRobotUndocked
from spot_bt.composites.sequence import create_undock_sequence


def create_generic_fiducial_selector(memory: bool = True) -> Selector:
    """Create a generic fiducial selector for Spot."""
    selector = Selector("Fiducial Search", memory=memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
        ]
    )

    return selector


def create_undock_selector(memory: bool = True) -> Selector:
    """Create an undock selector with a dock state check."""
    selector = Selector("Check and Undock", memory)
    selector.add_children(
        [
            IsRobotUndocked("Is Robot Undocked?"),
            create_undock_sequence(memory=memory),
        ]
    )

    return selector
