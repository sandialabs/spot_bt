"""Spot Inspecta Sequence composites."""
import py_trees

from spot_bt.behaviors.actions.general import RobotDock
from spot_bt.behaviors.actions.general import RobotUndock
from spot_bt.behaviors.actions.general import RobotPowerOff
from spot_bt.behaviors.actions.general import RobotPowerOn
from spot_bt.behaviors.actions.general import RobotState
from spot_bt.behaviors.actions.perception import DetectFiducialMarkers
from spot_bt.behaviors.actions.perception import TakeImage
from spot_bt.behaviors.conditions.perception import IsAnyFiducialMarkerDetected


def create_dock_sequence(name: str = "Dock and Turn Off", memory: bool = True) -> py_trees.composites.Sequence:
    """Create a generic Dock Sequence for Spot."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotState(name="Get Spot State"),
            RobotDock(name="Dock Spot"),
            RobotPowerOff(name="Power OFF Spot"),
        ]
    )

    return sequence


def create_exploration_sequence(name: str = "Explore", memory: bool = True) -> py_trees.composites.Sequence:
    """Create an exploration sequence for Spot."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotState(name="Get Spot State"),
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
        ]
    )

    return sequence


def create_find_dock_sequence(name: str = "Find Dock", memory: bool = True) -> py_trees.composites.Sequence:
    """Create a dock finding sequence for Spot."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children([
            RobotState(name="Get Spot State"),
            DetectFiducialMarkers(name="Detect")
        ]
    )

    return sequence


def create_move_to_target_sequence(name: str = "Explore", memory: bool = True) -> py_trees.composites.Sequence:
    """Create an exploration sequence for Spot."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotState(name="Get Spot State"),
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
        ]
    )

    return sequence


def create_undock_sequence(name: str = "Turn On and Undock", memory: bool = True) -> py_trees.composites.Sequence:
    """Create a generic Undock Sequence for Spot."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotPowerOn(name="Power ON Spot"),
            RobotUndock(name="Undock Spot"),
            RobotState(name="Get Spot State"),
        ]
    )

    return sequence
