from bosdyn.client.docking import blocking_dock_robot
from bosdyn.client.docking import blocking_undock
from bosdyn.client.robot_command import blocking_stand
from bosdyn.client.robot_command import RobotCommandClient

import py_trees

from spot_bt.data import Blackboards


class RobotDock(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.dock_id = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotDock::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="dock_id", access=py_trees.common.Access.READ
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.dock_id = int(self.blackboard.state.dock_id)

    def update(self) -> py_trees.common.Status:
        """Run the Docking behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotDock::update()]")
        try:
            # NOTE: Make sure the robot is powered on first!
            blocking_stand(self.client)
            blocking_dock_robot(self.robot, self.dock_id)
            self.logger.debug("\t\tROBOT DOCKED!")
        except:
            self.logger.debug("\t\tROBOT FAILED TO DOCK!")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotDock::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotUndock(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.dock_id = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotUndock::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("Spot State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the Docking behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotUndock::update()]")
        try:
            # NOTE: Make sure the robot is powered on first!
            blocking_undock(self.robot)
            self.logger.debug("\t\tROBOT UNDOCKED!")
        except:
            self.logger.debug("\t\tROBOT FAILED TO UNDOCK!")
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotUndock::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
