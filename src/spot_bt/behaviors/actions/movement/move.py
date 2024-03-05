"""Robot Motion behaviors."""
import time

from bosdyn.api import geometry_pb2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.api.spot import robot_command_pb2
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.frame_helpers import get_vision_tform_body
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.robot_command import RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient

import numpy as np

import py_trees

from spot_bt.data import Blackboards
from spot_bt.data import BodyTrajectory
from spot_bt.data import Pose
from spot_bt.utils import get_desired_angle
from spot_bt.utils import get_default_mobility_parameters


class MoveToFiducial(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = {}
        self.fiducials = None
        self.pose: Pose = None
        self.cmd_id = None
        self.dock_id = None
        self.state = None

    def initialise(self):
        """Initialize robot and client objects for behavior on first tick."""
        self.logger.debug(f"  {self.name} [MoveToFiducial::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="dock_id", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="state", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.state = self.blackboard.state.state
        self.dock_id = self.blackboard.state.dock_id
        self.fiducials = self.blackboard.perception.fiducials
        self.client = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the MoveToFiducial behavior when ticked."""
        self.logger.debug(f"  {self.name} [MoveToFiducial::update()]")
        if self.cmd_id is None:
            if len(self.fiducials) > 1:
                # Check which fiducial is not a dock
                for fiducial in self.fiducials:
                    if int(self.dock_id) != fiducial.apriltag_properties.tag_id:
                        print(f"IN HERE {self.dock_id}")
                        target_fiducial = fiducial
            else:
                target_fiducial = self.fiducials[0]

            # Get vision transform with respect to fiducial.
            vision_tform_fiducial = get_a_tform_b(
                target_fiducial.transforms_snapshot, VISION_FRAME_NAME,
                target_fiducial.apriltag_properties.frame_name_fiducial).to_proto()
            fiducial_rt_world = vision_tform_fiducial.position

            # Compute the go-to point with offsets
            robot_rt_world = get_vision_tform_body(self.state.kinematic_state.transforms_snapshot)
            robot_to_fiducial_ewrt_world = np.array([
                fiducial_rt_world.x - robot_rt_world.x,
                fiducial_rt_world.y - robot_rt_world.y,
                0,
            ])
            robot_to_fiducial_ewrt_world_norm = robot_to_fiducial_ewrt_world / np.linalg.norm(
                robot_to_fiducial_ewrt_world
            )
            heading = get_desired_angle(robot_to_fiducial_ewrt_world_norm)
            goto_rt_world = np.array([
                fiducial_rt_world.x - robot_to_fiducial_ewrt_world_norm[0] * 1.0,
                fiducial_rt_world.y - robot_to_fiducial_ewrt_world_norm[1] * 1.0,
            ])

            # Set mobility parameters
            # TODO: Make this into a static function possibly.
            mobility_parameters = get_default_mobility_parameters()

            # Create command and send
            cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=goto_rt_world[0], goal_y=goto_rt_world[1], goal_heading=heading,
                frame_name=VISION_FRAME_NAME, params=mobility_parameters,
                body_height=0.0, locomotion_hint=robot_command_pb2.HINT_AUTO
            )
            # TODO: Add a way of making the end_time_secs variable.
            self.cmd_id = self.client.robot_command(command=cmd, end_time_secs=time.time() + 10.0)

        # Go to the fiducial marker
        feedback = self.client.robot_command_feedback(self.cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
        # print(f"mobility: {mobility_feedback}")
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            self.logger.error("Failed to get to the fiducial marker!")
            return py_trees.common.Status.FAILURE

        trajectory_feedback = mobility_feedback.se2_trajectory_feedback
        # print(f"trajectory: {trajectory_feedback}")
        if (trajectory_feedback.status == trajectory_feedback.STATUS_AT_GOAL and
            trajectory_feedback.body_movement_status == trajectory_feedback.BODY_STATUS_SETTLED):
            self.logger.info("Reached the fiducial marker.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [MoveToFiducial::terminate().terminate()]" +
            f"[{self.status}->{new_status}]"
        )


class TurnInPlace(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = {}
        self.pose: Pose = None
        self.cmd_id = None

    def initialise(self):
        """Initialize robot and client objects for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TurnInPlace::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="pose", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.pose = self.blackboard.state.pose
        self.client["state"] = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.client["command"] = self.robot.ensure_client(RobotCommandClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the TurnInPlace behavior when ticked."""
        self.logger.debug(f"  {self.name} [TurnInPlace::update()]")
        try:
            if self.cmd_id is None:
                transform = self.client["state"].get_robot_state().kinematic_state.transforms_snapshot
                # TODO Need better way of inputing angle.
                cmd = BodyTrajectory().create_rotation_trajectory_command(
                    transform, 0, 0, 90
                )
                # TODO Figure out a way to vary time depending on angle size.
                self.cmd_id = self.client["command"].robot_command(
                    command=cmd, end_time_secs=(time.time() + 5)
                )

            feedback = self.client["command"].robot_command_feedback(self.cmd_id)
            mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
            if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
                self.logger.error("Failed to turn to desired angle!")
                return py_trees.common.Status.FAILURE

            trajectory_feedback = mobility_feedback.se2_trajectory_feedback
            if (trajectory_feedback.status == trajectory_feedback.STATUS_AT_GOAL and
                trajectory_feedback.body_movement_status == trajectory_feedback.BODY_STATUS_SETTLED):
                self.logger.info("Reached the desired angle.")
                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.RUNNING

        except:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [TurnInPlace::terminate().terminate()]" +
            f"[{self.status}->{new_status}]"
        )
