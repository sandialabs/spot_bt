"""spot_bt Arm Demonstration."""
from __future__ import annotations

import os
import time

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.math_helpers import SE3Pose

import py_trees
from py_trees.composites import Sequence

from spot_bt.data import Blackboards
from spot_bt.data import ArmPose, ArmPoses
from spot_bt.behaviors.actions.arm import ArmStow, ArmUnstow
from spot_bt.behaviors.actions.arm import CloseGripper, OpenGripper
from spot_bt.behaviors.actions.arm import ArmTrajectories
from spot_bt.composites.selector import create_undock_selector
from spot_bt.composites.sequence import create_dock_sequence
from spot_bt.tick import generic_pre_tick_handler


def create_root() -> Sequence:
    """Create the root for the Autonomy capability."""
    root = Sequence("DemoArm", memory=True)
    root.add_children(
        [
            create_undock_selector(),
            ArmUnstow(name="unstow_arm"),
            OpenGripper(name="open_gripper"),
            ArmTrajectories(name="move_arm_to_postions"),
            CloseGripper(name="close_gripper"),
            ArmStow(name="stow_arm"),
            create_dock_sequence(),
        ]
    )

    py_trees.display.render_dot_tree(root)

    return root


def main():
    sdk = bosdyn.client.create_standard_sdk("spot_bt_fiducial_demo")
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
    blackboard.state.register_key(
        key="is_gripper_open", access=py_trees.common.Access.WRITE
    )
    blackboard.state.is_gripper_open = False

    blackboard.arm = py_trees.blackboard.Client(name="Arm")
    blackboard.arm.register_key(key="command", access=py_trees.common.Access.WRITE)
    blackboard.arm.command = None
    blackboard.arm.register_key(key="target", access=py_trees.common.Access.WRITE)
    blackboard.arm.target = ArmPoses([
        ArmPose(
            SE3Pose(x=0.9, y=0.0, z=0.0, rot=[1.0, 0.0, 0.0, 0.0]),
            False,
        ),
        ArmPose(
            SE3Pose(x=0.9, y=0.5, z=0.0, rot=[1.0, 0.0, 0.0, 0.0]),
            False,
        ),
        ArmPose(
            SE3Pose(x=0.9, y=0.0, z=0.5, rot=[1.0, 0.0, 0.0, 0.0]),
            False,
        ),
    ])

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
        # behavior_tree.add_post_tick_handler(generic_post_tick_handler)
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
