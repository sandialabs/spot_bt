from bosdyn.client.robot_command import block_until_arm_arrives
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.robot_command import RobotCommandClient

import py_trees

from spot_bt.data import Blackboards


class ArmStow(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to stow manipulator arm."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.state = None
        self.option = "state"

    def setup(self, **kwargs):
        """Setup ArmStow behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmStow::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmStow::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the ArmStow behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmStow::update()]")
        try:
            # Build the stow command using RobotCommandBuilder
            stow = RobotCommandBuilder.arm_stow_command()

            # Issue the command via the RobotCommandClient
            stow_command_id = self.client.robot_command(stow)
            timeout = 3.0

            self.robot.logger.info("Stow command issued.")
            block_until_arm_arrives(self.client, stow_command_id, timeout)
        except:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ArmStow::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmUnstow(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to unstow manipulator arm."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = None
        self.robot = None
        self.client = None
        self.state = None
        self.option = "state"

    def setup(self, **kwargs):
        """Setup ArmUnstow behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmUnstow::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmUnstow::initialise()]")
        self.blackboard = self.attach_blackboard_client("State")
        self.blackboard.register_key(key="robot", access=py_trees.common.Access.WRITE)
        self.robot = self.blackboard.robot
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the ArmUnstow behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmUnstow::update()]")
        try:
            # Create an Unstow command
            unstow = RobotCommandBuilder.arm_ready_command()

            # Issue the command via the RobotCommandClient
            unstow_command_id = self.client.robot_command(unstow)
            timeout = 3.0

            # Blocks until arm achieves a finishing state for specific command
            self.robot.logger.info("Unstow command issued.")
            block_until_arm_arrives(self.client, unstow_command_id, timeout)
        except:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ArmUnstow::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
