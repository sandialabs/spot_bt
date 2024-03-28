"""Robot state related behaviors."""
from __future__ import annotations

from bosdyn.client.robot_state import RobotStateClient

import py_trees

from spot_bt.data import Blackboards


class RobotState(py_trees.behaviour.Behaviour):
    """Behavior state for getting robot's state."""

    _options = ["hardware", "metrics", "state"]

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.state = None
        self.option = "state"

    def setup(self, **kwargs):
        """Setup RobotState behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotState::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotState::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="state", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(RobotStateClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the RobotState behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotState::update()]")
        if self.option == "hardware":
            self.state = self.client.get_hardware_config_with_link_info()
            return py_trees.common.Status.SUCCESS
        if self.option == "metrics":
            self.state = self.client.get_robot_metrics()
            return py_trees.common.Status.SUCCESS
        if self.option == "state":
            self.state = self.client.get_robot_state()
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.blackboard.state.state = self.state
        self.logger.debug(
            f" {self.name} [RobotState::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotStateAsync(py_trees.behaviour.Behaviour):
    """Behavior state for getting robot's state. (ASYNC)"""

    _options = ["hardware", "metrics", "state"]

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards
        self.robot = None
        self.client = None
        self.state = None
        self.check = None
        self.option = "state"

    def setup(self, **kwargs):
        """Setup RobotStateAsync behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotStateAsync::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotStateAsync::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="state", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(RobotStateClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the RobotState behavior when ticked."""
        try:
            if self.check is None:
                self.logger.debug(f"  {self.name} [RobotStateAsync::update()]")
                if self.option == "hardware":
                    self.check = self.client.get_hardware_config_with_link_info_async()
                elif self.option == "metrics":
                    self.check = self.client.get_robot_metrics_async()
                elif self.option == "state":
                    self.check = self.client.get_robot_state_async()

            if not self.check.done():
                return py_trees.common.Status.RUNNING

            self.state = self.check.results()
            return py_trees.common.Status.SUCCESS

        except KeyError:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.blackboard.state.state = self.state
        self.logger.debug(
            f" {self.name} [RobotStateAsync::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class FinishAutonomy(py_trees.behaviour.Behaviour):
    """Behavior state for getting robot to finish overall autonomy."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.finished = None

    def setup(self, **kwargs):
        """Setup FinishAutonomy behavior before initialization."""
        self.logger.debug(f"  {self.name} [FinishAutonomy::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [FinishAutonomy::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="finished", access=py_trees.common.Access.WRITE
        )
        self.finished = self.blackboard.state.finished

    def update(self) -> py_trees.common.Status:
        """Run the FinishAutonomy behavior when ticked."""
        self.logger.debug(f"  {self.name} [FinishAutonomy::update()]")
        self.finished = True
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.blackboard.state.finished = self.finished
        self.logger.debug(
            f" {self.name} [FinishAutonomy::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
