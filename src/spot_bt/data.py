from __future__ import annotations

from dataclasses import dataclass, field

from bosdyn.api import (
    arm_command_pb2,
    geometry_pb2,
    synchronized_command_pb2,
    trajectory_pb2,
)
from bosdyn.api.spot import robot_command_pb2
from bosdyn.client.frame_helpers import get_a_tform_b, get_se2_a_tform_b
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose, Vec3
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.geometry import EulerZXY
from bosdyn.util import seconds_to_duration

import numpy as np

import py_trees


@dataclass
class Blackboards:
    """Dataclass for various BT Blackboards related to spot_inspecta."""

    state: py_trees.blackboard.Client | None = field(default=None)
    arm: py_trees.blackboard.Client | None = field(default=None)
    graph: py_trees.blackboard.Client | None = field(default=None)
    perception: py_trees.blackboard.Client | None = field(default=None)
    ins: py_trees.blackboard.Client | None = field(default=None)


@dataclass
class Pose:
    """Dataclass for pose in free space, composed of position and orientation."""

    position: geometry_pb2.Vec3 = field(
        default_factory=lambda: geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    )
    orientation: geometry_pb2.Quaternion = field(
        default_factory=lambda: geometry_pb2.Quaternion(
            w=0.0, x=0.0, y=0.0, z=0.0
        )
    )
    pose: EulerZXY = field(default_factory=lambda: EulerZXY(yaw=0.0, roll=0.0, pitch=0.0))
    body_height: float = field(default=0.0)
    is_done: bool = field(default=False)

    def as_list(self) -> list[float | int]:
        """Get values as a Python List."""
        return [
            self.position.x,
            self.position.y,
            self.position.z,
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w,
            self.pose.pitch,
            self.pose.roll,
            self.pose.yaw,
            self.body_height,
        ]

    def mark(self):
        """Set that the pose stored has been completed."""
        self.is_done = True

    def unmark(self):
        """Set that the pose stored has not been completed."""
        self.is_done = False

    def set_orientation(
        self, w: float = 0.0, x: float = 0.0, y: float = 0.0, z: float = 0.0
    ):
        """Set new pose orientation for Spot to perform."""
        self.orientation.w = w
        self.orientation.x = x
        self.orientation.y = y
        self.orientation.z = z
        self.unmark()

    def set_pose(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        body_height: float = 0.0,
    ):
        """Set standing pose of the robot."""
        self.pose.roll = roll
        self.pose.pitch = pitch
        self.pose.yaw = yaw
        self.body_height = body_height
        self.unmark()

    def set_position(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """Set new pose position for Spot to perform."""
        self.position.x = x
        self.position.y = y
        self.position.z = z
        self.unmark()

    def create_command(self) -> RobotCommandBuilder:
        """Create a pose command to pass to client."""
        return RobotCommandBuilder.synchro_stand_command(
            footprint_R_body=self.pose, body_height=self.body_height
        )


@dataclass
class Movement:
    """Dataclass for movement in free space."""

    v_x: float = field(default=0.0)
    v_y: float = field(default=0.0)
    v_rot: float = field(default=0.0)
    is_done: bool = field(default=False)

    def as_list(self) -> list[float | int]:
        """Get values as a Python List."""
        return [
            self.v_x,
            self.v_y,
            self.v_rot,
        ]

    def mark(self):
        """Set that the pose stored has been completed."""
        self.is_done = True

    def unmark(self):
        """Set that the pose stored has not been completed."""
        self.is_done = False

    def set_velocity(self, x: float = 0.0, y: float = 0.0, rot: float = 0.0):
        """Set new movement velocity for Spot to perform."""
        self.v_x = x
        self.v_y = y
        self.v_rot = rot
        self.unmark()

    def create_command(self) -> RobotCommandBuilder:
        """Create a movement command to pass to client."""
        return RobotCommandBuilder.synchro_velocity_command(
            v_x=self.v_x, v_y=self.v_y, v_rot=self.v_rot
        )


@dataclass
class BodyTrajectory:
    """Dataclass for a Spot movement trajectory."""

    pose: SE2Pose | SE3Pose = field(
        default_factory=lambda: SE3Pose(x=0.0, y=0.0, z=0.0, rot=0.0)
    )

    def mark(self):
        """Set that the pose stored has been completed."""
        self.is_done = True

    def unmark(self):
        """Set that the pose stored has not been completed."""
        self.is_done = False

    def set_pose(self, dx: float, dy: float, dz: float, rot: float):
        """Set new body pose for Spot."""
        self.pose.x = dx
        self.pose.y = dy
        self.pose.z = dz
        self.pose.rot = rot
        self.unmark()

    def create_rotation_trajectory_command(
        self, transform, angle: float, frame_type: str = "vision"
    ) -> RobotCommandBuilder:
        """Create an in-place rotation trajectory to pass to the client."""
        # if isinstance(self.pose, SE3Pose):
        #     raise TypeError("Rotations require use of SE2Pose not SE3Pose")
        if frame_type not in ["odom", "vision"]:
            frame = VISION_FRAME_NAME
        else:
            frame = frame_type

        body_tform_goal = SE2Pose(x=0.0, y=0.0, angle=angle)
        frame_tform_body = get_se2_a_tform_b(
            transform, frame, GRAV_ALIGNED_BODY_FRAME_NAME
        )
        frame_tform_goal = frame_tform_body * body_tform_goal

        return RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=frame_tform_goal.x,
            goal_y=frame_tform_goal.y,
            goal_heading=frame_tform_goal.angle,
            frame_name=frame,
            params=RobotCommandBuilder.mobility_params(stair_hint=False),
        )

    def create_point_trajectory_command(
        self,
        pose_snapshot,
        mobility_parameters: robot_command_pb2.MobilityParams | None = None,
    ) -> RobotCommandBuilder:
        """Create a point trajectory command to a target."""

    def create_full_trajectory_command(
        self, pose_snapshot, target_pose: SE3Pose | Vec3
    ) -> RobotCommandBuilder:
        """Create a full body trajectory to pass to client."""
        self.pose = get_a_tform_b(
            pose_snapshot,
            frame_a=VISION_FRAME_NAME,
            frame_b=GRAV_ALIGNED_BODY_FRAME_NAME,
        )

        # Compute vector between Spot and target
        robot_rt_target_ewrt_vision = [
            self.pose.x - target_pose.x,
            self.pose.y - target_pose.y,
            self.pose.z - target_pose.z,
        ]

        # Compute the unit vector
        if np.linalg.norm(robot_rt_target_ewrt_vision) < 0.01:  # if too close
            robot_rt_target_ewrt_vision_hat = self.pose.transform_point(1, 0, 0)
        else:
            robot_rt_target_ewrt_vision_hat = (
                robot_rt_target_ewrt_vision
                / np.linalg.norm(robot_rt_target_ewrt_vision)
            )


@dataclass
class ArmPose:
    """Dataclass for a single Spot Arm Pose."""

    pose: SE3Pose = field(default_factory=lambda: SE3Pose(x=0.0, y=0.0, z=0.0, rot=Quat()))
    is_done: bool = field(default=False)

    def hand_pose(self) -> geometry_pb2.Vec3:
        """Get the hand pose relative to target."""
        return geometry_pb2.Vec3(
            x=self.pose.x, y=self.pose.y, z=self.pose.z
        )

    def hand_orientation(self) -> geometry_pb2.Quaternion:
        """Get the hand orientation relative to target."""
        return geometry_pb2.Quaternion(
            w=self.pose.rot[0], x=self.pose.rot[1], y=self.pose.rot[2], z=self.pose.rot[3]
        )


@dataclass
class ArmPoses:
    """Dataclass for multiple poses that constitute a Spot/Arm trajectory."""

    poses: list[SE3Pose | ArmPose] = field(
        default_factory=lambda: SE3Pose(x=0.0, y=0.0, z=0.0, rot=Quat())
    )
    is_done: bool = field(default=False)

    def mark(self):
        """Set that the pose stored has been completed."""
        self.is_done = True

    def unmark(self):
        """Set that the pose stored has not been completed."""
        self.is_done = False

    def set_poses(self, poses: list[SE3Pose]):
        """Set a set of arm poses to constitute a trajectory."""
        self.poses = poses

    def specific_pose(self, i: int) -> SE3Pose:
        """Get specific pose from list."""
        try:
            return self.poses[i]

        except IndexError:
            return None

    def size(self) -> int:
        """Return size of trajectory."""
        return len(self.poses)

    def create_command(
        self, seconds: int = 2, gripper_open_fraction: float = 0.0
    ) -> RobotCommandBuilder:
        """Create an arm trajectory command to pass to client."""
        # pylint: disable=no-member
        trajectories = []
        for pose in self.poses:
            trajectories.append(
                trajectory_pb2.SE3TrajectoryPoint(
                    pose=pose, time_since_reference=seconds_to_duration(seconds)
                )
            )

        # Build trajectory proto by combining the points.
        full_trajectory = trajectory_pb2.SE3Trajectory(points=trajectories)
        arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
            pose_trajectory_in_task=full_trajectory,
            root_frame_name=GRAV_ALIGNED_BODY_FRAME_NAME,
        )
        arm_command = arm_command_pb2.ArmCommand.Request(
            arm_cartesian_command=arm_cartesian_command
        )
        synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
            arm_command=arm_command
        )
        robot_command = robot_command_pb2.RobotCommand(
            synchronized_command=synchronized_command
        )

        # Keep the gripper at gripper_open_fraction value the whole time.
        robot_command = RobotCommandBuilder.claw_gripper_open_fraction_command(
            gripper_open_fraction, build_on_command=robot_command
        )
