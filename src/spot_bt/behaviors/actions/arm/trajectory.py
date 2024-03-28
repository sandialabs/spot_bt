from __future__ import annotations

from bosdyn.api import arm_command_pb2, geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import (
    get_a_tform_b,
    GRAV_ALIGNED_BODY_FRAME_NAME,
    ODOM_FRAME_NAME,
)
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

import py_trees

from spot_bt.data import ArmPose, ArmPoses, Blackboards


class ArmTrajectory(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to move manipulator arm to a single point."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.cmd_client = None
        self.state_client = None
        self.feedback = None
        self.target: ArmPose | None = None
        self.cmd_id = None

    def setup(self, **kwargs):
        """Setup ArmTrajectory behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(key="robot", access=py_trees.common.Access.WRITE)
        self.blackboard.state.register_key(
            key="is_gripper_open", access=py_trees.common.Access.READ
        )
        self.blackboard.arm = self.attach_blackboard_client("Arm")
        self.blackboard.arm.register_key(
            key="target", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.target = self.blackboard.arm.target
        self.cmd_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the ArmTrajectory behavior when ticked."""
        # pylint: disable=no-member
        self.logger.debug(f"  {self.name} [ArmTrajectory::update()]")
        try:
            if isinstance(self.target, ArmPoses):
                target = self.target.poses[0]
                self.logger.warning("Multiple poses given to ArmTrajector.")
            else:
                target = self.target

            if self.feedback is None:
                # Make the arm pose RobotCommand
                # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).
                hand_ewrt_flat_body = target.hand_pose()

                # Rotation as a quaternion
                flat_body_Q_hand = target.hand_orientation()

                flat_body_T_hand = geometry_pb2.SE3Pose(
                    position=hand_ewrt_flat_body, rotation=flat_body_Q_hand
                )

                robot_state = self.state_client.get_robot_state()
                odom_T_flat_body = get_a_tform_b(
                    robot_state.kinematic_state.transforms_snapshot,
                    ODOM_FRAME_NAME,
                    GRAV_ALIGNED_BODY_FRAME_NAME,
                )

                odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(
                    flat_body_T_hand
                )

                # duration in seconds
                seconds = 2  # TODO Consider making this a parameter/config

                arm_command = RobotCommandBuilder.arm_pose_command(
                    odom_T_hand.x,
                    odom_T_hand.y,
                    odom_T_hand.z,
                    odom_T_hand.rot.w,
                    odom_T_hand.rot.x,
                    odom_T_hand.rot.y,
                    odom_T_hand.rot.z,
                    ODOM_FRAME_NAME,
                    seconds,
                )

                # Make the open gripper RobotCommand
                if self.blackboard.state.is_gripper_open:
                    gripper_command = (
                        RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
                    )
                else:
                    gripper_command = RobotCommandBuilder.claw_gripper_close_command()

                # Combine the arm and gripper commands into one RobotCommand
                command = RobotCommandBuilder.build_synchro_command(
                    gripper_command, arm_command
                )

                # Send the request
                self.cmd_id = self.cmd_client.robot_command(command)

            # Wait until the arm arrives at the goal.
            self.feedback = self.cmd_client.robot_command_feedback(self.cmd_id)
            self.logger.info(
                "Distance to go: "
                f"{self.feedback.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_pos_distance_to_goal:.2f} meters"
                f", {self.feedback.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_rot_distance_to_goal:.2f} radians"
            )

            if (
                self.feedback.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status
                == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE
            ):
                self.logger.info("Move complete.")
                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.RUNNING

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ArmTrajectory::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmTrajectories(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to move manipulator arm to multiple points."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.cmd_client = None
        self.state_client = None
        self.feedback = None
        self.target: ArmPoses | None = None
        self.current_target = None
        self.possible_targets = None
        self.count = 0
        self.cmd_id = None

    def setup(self, **kwargs):
        """Setup ArmTrajectories behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmTrajectories::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmTrajectories::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(key="robot", access=py_trees.common.Access.WRITE)
        self.blackboard.arm = self.attach_blackboard_client("Arm")
        self.blackboard.arm.register_key(
            key="target", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.target = self.blackboard.arm.target
        self.cmd_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.count = 0
        if isinstance(self.target, ArmPose):
            self.possible_targets = [self.target.pose]
            self.logger.warning("Only one pose given to ArmTrajectories")
        else:
            self.possible_targets = self.target.poses

    def update(self) -> py_trees.common.Status:
        """Run the ArmTrajectories behavior when ticked."""
        # pylint: disable=no-member
        self.logger.debug(f"  {self.name} [ArmTrajectories::update()]")
        try:
            if self.feedback is None:
                self.current_target = self.possible_targets[self.count]
                # Make the arm pose RobotCommand
                # Build a position to move the arm to (in meters, relative to and expressed in the
                # gravity aligned body frame).
                hand_ewrt_flat_body = self.current_target.hand_pose()

                # Rotation as a quaternion
                flat_body_Q_hand = self.current_target.hand_orientation()

                flat_body_T_hand = geometry_pb2.SE3Pose(
                    position=hand_ewrt_flat_body, rotation=flat_body_Q_hand
                )

                robot_state = self.state_client.get_robot_state()
                odom_T_flat_body = get_a_tform_b(
                    robot_state.kinematic_state.transforms_snapshot,
                    ODOM_FRAME_NAME,
                    GRAV_ALIGNED_BODY_FRAME_NAME,
                )

                odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(
                    flat_body_T_hand
                )

                # duration in seconds
                seconds = 2  # TODO Consider making this a parameter/config

                arm_command = RobotCommandBuilder.arm_pose_command(
                    odom_T_hand.x,
                    odom_T_hand.y,
                    odom_T_hand.z,
                    odom_T_hand.rot.w,
                    odom_T_hand.rot.x,
                    odom_T_hand.rot.y,
                    odom_T_hand.rot.z,
                    ODOM_FRAME_NAME,
                    seconds,
                )

                # Make the open gripper RobotCommand
                # TODO Fix this. This should be what the current state of the gripper is.
                gripper_command = (
                    RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
                )

                # Combine the arm and gripper commands into one RobotCommand
                command = RobotCommandBuilder.build_synchro_command(
                    gripper_command, arm_command
                )

                # Send the request
                self.cmd_id = self.cmd_client.robot_command(command)

            # Wait until the arm arrives at the goal.
            self.feedback = self.cmd_client.robot_command_feedback(self.cmd_id)
            self.logger.info(
                "Distance to go: "
                f"{self.feedback.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_pos_distance_to_goal:.2f} meters"
                f", {self.feedback.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.measured_rot_distance_to_goal:.2f} radians"
            )

            if (
                self.feedback.feedback.synchronized_feedback.arm_command_feedback.arm_cartesian_feedback.status
                == arm_command_pb2.ArmCartesianCommand.Feedback.STATUS_TRAJECTORY_COMPLETE
            ):
                self.logger.info("Move complete.")
                self.count += 1
                self.feedback = None

                if self.count >= len(self.possible_targets):
                    self.logger.info("All moves complete!")
                    return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.RUNNING

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ArmTrajectories::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
