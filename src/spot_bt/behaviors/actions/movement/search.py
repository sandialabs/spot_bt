"""Robot Search behaviors."""
from __future__ import annotations

import random
import time

from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

import py_trees

from spot_bt.data import Blackboards, BodyTrajectory
from spot_bt.utils import get_default_mobility_parameters


class RandomSimpleSearch(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = {}
        self.cmd_id = None
        self.dock_id = None
        self.state = None
        self.move_duration = None
        self.is_rotating = True
        self.is_moving = False
        self.done = False

    def initialise(self):
        """Initialize robot and client objects for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RandomSearch::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="state", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="move_duration", access=py_trees.common.Access.READ
        )
        self.robot = self.blackboard.state.robot
        self.state = self.blackboard.state.state
        self.move_duration = self.blackboard.state.move_duration
        self.client["command"] = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.client["state"] = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.is_rotating = True
        self.is_moving = False
        self.cmd_id = None

    def update(self) -> py_trees.common.Status:
        """Run the RandomSearch behavior when ticked."""
        # pylint: disable=no-member
        self.logger.debug(f"  {self.name} [RandomSearch::update()]")
        if self.cmd_id is None and self.is_moving:
            # Set mobility parameters
            # TODO: Make this into a static function possibly.
            mobility_parameters = get_default_mobility_parameters()

            # Create command and send
            cmd = RobotCommandBuilder.synchro_velocity_command(
                v_x=0.5, v_y=0.0, v_rot=0.0, params=mobility_parameters
            )
            self.cmd_id = self.client["command"].robot_command(
                command=cmd, end_time_secs=(time.time() + self.move_duration)
            )

        if self.cmd_id is None and self.is_rotating:
            transform = self.client["state"].get_robot_state().kinematic_state.transforms_snapshot
            cmd = BodyTrajectory().create_rotation_trajectory_command(
                transform, 45.0 * random.uniform(-1.0, 1.0), ODOM_FRAME_NAME
            )
            self.cmd_id = self.client["command"].robot_command(
                command=cmd, end_time_secs=(time.time() + self.move_duration)
            )

        feedback = self.client["command"].robot_command_feedback(self.cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            self.logger.error(f"Failed to search! Status: {mobility_feedback.status}")
            return py_trees.common.Status.FAILURE

        trajectory_feedback = mobility_feedback.se2_trajectory_feedback
        if (trajectory_feedback.status == trajectory_feedback.STATUS_AT_GOAL and
            trajectory_feedback.body_movement_status == trajectory_feedback.BODY_STATUS_SETTLED):
            self.logger.info("Finished searching!")
            if self.is_rotating:
                self.is_moving = True
                self.is_rotating = False
                self.cmd_id = None
            else:
                self.is_moving = False
                self.is_rotating = False
                self.cmd_id = None
                self.done = True

        if self.done:
            # TODO: Change this to a success status
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [RandomSearch::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
