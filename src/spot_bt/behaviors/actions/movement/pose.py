"""Robot Pose related behaviors"""
from __future__ import annotations

import time

from bosdyn.client.robot_command import RobotCommandClient

import py_trees

from spot_bt.data import Blackboards


class RobotPose(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to pose."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.state = None
        self.pose = None

    def setup(self, **kwargs):
        """Setup RobotPose behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotPose::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [RobotPose::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="pose", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.pose = self.blackboard.state.pose
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the RobotPose behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotPose::update()]")
        try:
            assert self.robot.is_powered_on(), "Robot power on failed."
            cmd = self.pose.create_command()
            self.client.robot_command(cmd)
            self.pose.mark()
            time.sleep(3)

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.state.pose = self.pose
        self.logger.debug(
            f" {self.name} [RobotPose::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
