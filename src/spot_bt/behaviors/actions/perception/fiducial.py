from bosdyn.api import world_object_pb2
from bosdyn.client.world_object import WorldObjectClient

import py_trees

from spot_bt.data import Blackboards


class DetectFiducialMarkers(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.client = self.robot.ensure_client(WorldObjectClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::update()]")
        try:
            request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
            self.fiducials = self.client.list_world_objects(
                object_type=request_fiducials
            ).world_objects
            print(self.fiducials)
        except:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.fiducials = self.fiducials
        self.logger.debug(
            f" {self.name} [DetectFiducialMarkers::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
