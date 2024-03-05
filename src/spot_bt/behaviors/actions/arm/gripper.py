from bosdyn.client.robot_command import block_until_arm_arrives
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.robot_command import RobotCommandClient

import py_trees

from spot_bt.data import Blackboards


class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None

    def setup(self, **kwargs):
        """Setup CloseGripper behavior before initialization."""
        self.logger.debug(f"  {self.name} [CloseGripper::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [CloseGripper::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the CloseGripper behavior when ticked."""
        self.logger.debug(f"  {self.name} [CloseGripper::update()]")
        try:
            # Build the stow command using RobotCommandBuilder
            command = RobotCommandBuilder.claw_gripper_close_command()

            # Issue the command via the RobotCommandClient
            command_id = self.client.robot_command(command)
            timeout = 3.0

            self.robot.logger.info("Close gripper command issued.")
            block_until_arm_arrives(self.client, command_id, timeout)
        except:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [CloseGripper::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None

    def setup(self, **kwargs):
        """Setup OpenGripper behavior before initialization."""
        self.logger.debug(f"  {self.name} [OpenGripper::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [OpenGripper::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)


    def update(self) -> py_trees.common.Status:
        """Run the OpenGripper behavior when ticked."""
        self.logger.debug(f"  {self.name} [OpenGripper::update()]")
        try:
            # Build the stow command using RobotCommandBuilder
            command = RobotCommandBuilder.claw_gripper_open_command()

            # Issue the command via the RobotCommandClient
            command_id = self.client.robot_command(command)
            timeout = 3.0

            self.robot.logger.info("Open gripper command issued.")
            block_until_arm_arrives(self.client, command_id, timeout)
        except:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [OpenGripper::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
