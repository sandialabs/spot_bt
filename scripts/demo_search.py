"""spot_bt Search Demonstration"""
from __future__ import annotations

import os
import time

import bosdyn.client
import bosdyn.client.util

import py_trees
from py_trees.composites import Selector, Sequence

from spot_bt.data import Blackboards
from spot_bt.behaviors.actions.general import RobotState
from spot_bt.behaviors.actions.movement import MoveToFiducial, RandomSimpleSearch
from spot_bt.behaviors.actions.perception import DetectFiducialMarkers
from spot_bt.behaviors.conditions.perception import IsAnyFiducialMarkerDetected
from spot_bt.composites.selector import create_undock_selector
from spot_bt.composites.sequence import create_dock_sequence
from spot_bt.tick import generic_pre_tick_handler


class IsFiducialMoveComplete(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.fiducial_move_complete = False

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"  {self.name} [IsFiducialMoveComplete::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="fiducial_move_complete", access=py_trees.common.Access.READ
        )
        self.fiducial_move_complete = self.blackboard.state.fiducial_move_complete

    def update(self):
        """Run the IsFiducialMoveComplete condition when ticked."""
        self.logger.debug(f"  {self.name} [IsFiducialMoveComplete::update()]")
        if self.fiducial_move_complete:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f" {self.name} [IsFiducialmoveComplete::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class MarkFiducialMoveComplete(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.fiducial_move_complete = False

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"  {self.name} [MarkFiducialMoveComplete::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="fiducial_move_complete", access=py_trees.common.Access.WRITE
        )

    def update(self):
        """Run the IsFiducialMoveComplete condition when ticked."""
        self.logger.debug(f"  {self.name} [MarkFiducialMoveComplete::update()]")
        self.blackboard.state.fiducial_move_complete = True
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f" {self.name} [MarkFiducialmoveComplete::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


def create_detect_and_move_sb(
    name="Move to Marker, if Detected", memory: bool = False
) -> Sequence:
    """Create a Fiducial Detection Sub-Behavior"""
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            DetectFiducialMarkers("Detect Fiducial Marker"),
            IsAnyFiducialMarkerDetected("Is Any Fiducial Marker Detected?"),
            MoveToFiducial("Move to Fiducial"),
            MarkFiducialMoveComplete("Mark Completed"),
        ]
    )
    return sequence


def create_search_sb(name="Search for Fiducial", memory: bool = False) -> Sequence:
    """Create Search Sub-Behavior."""
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            RobotState("Get Spot State"),
            RandomSimpleSearch("Search Area"),
        ]
    )
    return sequence


def create_move_to_fiducial_ppa(
        name: str = "Move to Fiducial PPA", memory: bool = False
) -> Selector:
    """Create Move to Fiducial Precondition-Postcondition-Action."""
    ppa = Selector(name, memory)
    ppa.add_children(
        [
            IsFiducialMoveComplete("Is Move to Fiducial Complete?"),
            create_detect_and_move_sb(),
            create_search_sb(),
        ]
    )

    return ppa


def create_root() -> Sequence:
    """Create the root for the Autonomy capability."""
    root = Sequence("DemoSearch", memory=True)
    root.add_children(
        [
            create_undock_selector(),
            create_move_to_fiducial_ppa(),
            create_dock_sequence(),
        ]
    )
    py_trees.display.render_dot_tree(root)

    return root


def main():
    # Set up robot
    sdk = bosdyn.client.create_standard_sdk("spot_bt_autonomy_demo")
    spot_ip = os.getenv("SPOT_IP")
    if spot_ip is None:
        raise ValueError(
            "The SPOT_IP environment variable was not set."
        )
    robot = sdk.create_robot(spot_ip)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    # Create behavior tree and blackboard
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
    blackboard = Blackboards()
    blackboard.state = py_trees.blackboard.Client(name="State")
    blackboard.state.register_key(key="robot", access=py_trees.common.Access.WRITE)
    blackboard.state.robot = robot
    blackboard.state.register_key(key="dock_id", access=py_trees.common.Access.WRITE)
    blackboard.state.dock_id = 549
    blackboard.state.register_key(key="is_docked", access=py_trees.common.Access.WRITE)
    blackboard.state.is_docked = True
    blackboard.state.register_key(key="move_duration", access=py_trees.common.Access.WRITE)
    blackboard.state.move_duration = 3.0
    blackboard.state.register_key(
        key="fiducial_move_complete", access=py_trees.common.Access.WRITE
    )
    blackboard.state.fiducial_move_complete = False

    blackboard.perception = py_trees.blackboard.Client(name="Perception")
    blackboard.perception.register_key(
        key="fiducials", access=py_trees.common.Access.WRITE
    )
    blackboard.perception.fiducials = None

    # Create and execute behavior tree
    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name
    )
    with bosdyn.client.lease.LeaseKeepAlive(
        lease_client, must_acquire=True, return_at_exit=True
    ):
        # Create visitors
        debug_visitor = py_trees.visitors.DebugVisitor()

        # Enable tree stewardship
        root = create_root()
        behavior_tree = py_trees.trees.BehaviourTree(root)
        behavior_tree.add_pre_tick_handler(generic_pre_tick_handler)
        behavior_tree.visitors.append(debug_visitor)
        behavior_tree.setup(timeout=15)
        root.setup_with_descendants()

        # Begin autonomy loop
        while True:
            try:
                behavior_tree.tick()
                time.sleep(0.1)
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    main()
