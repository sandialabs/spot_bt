"""spot_bt Pose Demonstration."""
from __future__ import annotations

import os

import bosdyn.client
import bosdyn.client.util

import py_trees

from spot_bt.data import Pose
from spot_bt.behaviors.actions.general import ClaimLease
from spot_bt.behaviors.actions.general import RobotDock
from spot_bt.behaviors.actions.general import RobotPowerOff
from spot_bt.behaviors.actions.general import RobotPowerOn
from spot_bt.behaviors.actions.general import RobotUndock
from spot_bt.behaviors.actions.movement import RobotPose


def create_root() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence("DemoPose", memory=True)
    claim_robot = ClaimLease(name="Claim Lease")
    power_on_robot = RobotPowerOn(name="Power ON Spot")
    undock_robot = RobotUndock(name="Undock Spot")
    set_robot_pose = RobotPose(name="Spot Pose")
    dock_robot = RobotDock(name="Dock Spot")
    power_off_robot = RobotPowerOff(name="Power OFF Spot")
    root.add_children(
        [
            claim_robot,
            power_on_robot,
            undock_robot,
            set_robot_pose,
            dock_robot,
            power_off_robot,
        ]
    )
    return root


def main():
    sdk = bosdyn.client.create_standard_sdk("spot_bt_pose_demo")
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
    blackboard = py_trees.blackboard.Client(name="State")
    blackboard.register_key(key="robot", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="dock_id", access=py_trees.common.Access.WRITE)
    blackboard.robot = robot
    blackboard.pose = Pose()
    blackboard.pose.set_pose(yaw=0.4, roll=0.0, pitch=0.0)
    blackboard.dock_id = 549

    # Create and execute behavior tree
    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name
    )
    with bosdyn.client.lease.LeaseKeepAlive(
        lease_client, must_acquire=True, return_at_exit=True
    ):
        root = create_root()
        root.setup_with_descendants()
        for i in range(1):
            print(f"\n------- Tick {i} -------")
            root.tick_once()
            print(py_trees.display.unicode_tree(root=root, show_status=True))


if __name__ == "__main__":
    main()
