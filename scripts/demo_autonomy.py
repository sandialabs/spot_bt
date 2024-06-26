"""spot_bt Autonomy Demonstration."""
from __future__ import annotations

import os
import time

import bosdyn.client
import bosdyn.client.util

import py_trees

from spot_bt.data import Blackboards, Pose
from spot_bt.behaviors.actions.general import RobotState
from spot_bt.behaviors.actions.movement import MoveToFiducial, RobotPose, TurnInPlace
from spot_bt.behaviors.actions.perception import DetectFiducialMarkers, DetectWorldObjects
from spot_bt.composites.selector import create_generic_fiducial_selector, create_undock_selector
from spot_bt.composites.sequence import create_dock_sequence
from spot_bt.tick import generic_pre_tick_handler


def create_root() -> py_trees.composites.Sequence:
    """Create the root for the Autonomy capability."""
    root = py_trees.composites.Sequence("DemoAutonomy", memory=True)
    root.add_children(
        [
            create_undock_selector(),
            RobotState(name="Get Spot State"),
            RobotPose(name="Set Spot Pose"),
            TurnInPlace("Turn   0 ->  90"),
            TurnInPlace("Turn  90 -> 180"),
            TurnInPlace("Turn 180 -> 270"),
            TurnInPlace("Turn 270 -> 360"),
            DetectWorldObjects(name="Detect World Objects"),
            DetectFiducialMarkers(name="Detect Fiducials"),
            create_generic_fiducial_selector(),
            RobotState(name="Get Spot State"),
            MoveToFiducial(name="Move to Fiducial"),
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
    else:
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
    blackboard.state.register_key(key="pose", access=py_trees.common.Access.WRITE)
    blackboard.state.pose = Pose()
    blackboard.state.pose.set_pose(yaw=0.4, roll=0.0, pitch=0.0)
    blackboard.state.register_key(key="is_docked", access=py_trees.common.Access.WRITE)
    blackboard.state.is_docked = True

    blackboard.perception = py_trees.blackboard.Client(name="Perception")
    blackboard.perception.register_key(
        key="fiducials", access=py_trees.common.Access.WRITE
    )
    blackboard.perception.fiducials = None
    blackboard.perception.register_key(
        key="world_objects", access=py_trees.common.Access.WRITE
    )
    blackboard.perception.world_objects = None

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
