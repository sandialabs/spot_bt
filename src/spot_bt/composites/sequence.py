"""spot_bt Sequence composites."""
from __future__ import annotations

from py_trees.composites import Sequence

from spot_bt.behaviors.actions.general import (
    RobotDock,
    RobotUndock,
    RobotPowerOff,
    RobotPowerOn,
    RobotState,
)
from spot_bt.behaviors.actions.perception import DetectFiducialMarkers
from spot_bt.behaviors.conditions.perception import IsAnyFiducialMarkerDetected


def create_dock_sequence(name: str = "Dock and Turn Off", memory: bool = True) -> Sequence:
    """Create a generic Dock Sequence for Spot."""
    sequence = Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotState(name="Get Spot State"),
            RobotDock(name="Dock Spot"),
            RobotPowerOff(name="Power OFF Spot"),
        ]
    )

    return sequence


def create_undock_sequence(name: str = "Turn On and Undock", memory: bool = True) -> Sequence:
    """Create a generic Undock Sequence for Spot."""
    sequence = Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotPowerOn(name="Power ON Spot"),
            RobotUndock(name="Undock Spot"),
            RobotState(name="Get Spot State"),
        ]
    )

    return sequence
