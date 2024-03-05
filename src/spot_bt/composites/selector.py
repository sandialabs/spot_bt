"""Spot Inspecta Selector composites."""
import py_trees

from spot_bt.behaviors.actions.perception import DetectFiducialMarkers
from spot_bt.behaviors.actions.perception import SwitchCamera
from spot_bt.behaviors.actions.perception import TakeImage
from spot_bt.behaviors.conditions.perception import IsAnyFiducialMarkerDetected
from spot_bt.behaviors.conditions.perception import IsDockFiducialMarkerDetected
from spot_bt.composites.sequence import create_exploration_sequence


def create_fiducial_search_selector(memory: bool = True) -> py_trees.composites.Selector:
    """Create a generic fiducial search selector for Spot."""
    selector = py_trees.composites.Selector("Fiducial Search", memory=memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
            create_exploration_sequence(),
        ]
    )

    return selector


def create_dock_search_selector(memory: bool = True) -> py_trees.composites.Selector:
    """Create a generic fiducial search selector for Spot."""
    selector = py_trees.composites.Selector("Fiducial Search", memory=memory)
    selector.add_child(
        IsDockFiducialMarkerDetected(name="Is Dock Fiducial Detected?")
    )
    selector.add_child(create_exploration_sequence())
    return selector


def create_generic_fiducial_selector(memory: bool = True) -> py_trees.composites.Selector:
    """Create a generic fiducial selector for Spot."""
    selector = py_trees.composites.Selector("Fiducial Search", memory=memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
        ]
    )

    return selector
