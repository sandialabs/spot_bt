"""Spot Docking Related Conditions."""
from __future__ import annotations

import py_trees

from spot_bt.data import Blackboards


class IsRobotUndocked(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.is_docked = False

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"  {self.name} [IsRobotUndocked::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="is_docked", access=py_trees.common.Access.READ
        )
        self.is_docked = self.blackboard.state.is_docked

    def update(self):
        """Run the IsRobotUndocked condition when ticked."""
        self.logger.debug(f"  {self.name} [IsRobotUndocked::update()]")
        if self.is_docked:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f" {self.name} [IsRobotUndocked::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
