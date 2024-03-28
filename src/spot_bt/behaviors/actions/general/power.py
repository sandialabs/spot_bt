"""Robot power function related behaviors"""
from __future__ import annotations

import py_trees

from spot_bt.data import Blackboards


class RobotPowerOn(py_trees.behaviour.Behaviour):
    """Behavior class for powering-on robot."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def setup(self, **kwargs):
        """Setup RobotPowerOn behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotPowerOn::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotPose::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("Spot State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the PowerOn behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotPose::update()]")
        try:
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot power on failed!"
        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotPowerOn::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotPowerOff(py_trees.behaviour.Behaviour):
    """Behavior class for powering-off robot."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def setup(self, **kwargs):
        """Setup RobotPowerOff behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotPowerOff::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotPose::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the PowerOff behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotPowerOff::update()]")
        try:
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            assert not self.robot.is_powered_on(), "Robot power off failed!"
        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotPowerOff::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
