from __future__ import annotations

from bosdyn.client.world_object import WorldObjectClient

import py_trees

from spot_bt.data import Blackboards


class DetectWorldObjects(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.client = None
        self.world_objects = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def setup(self, **kwargs):
        """Setup DetectWorldObjects behavior."""
        self.logger.debug(f"  {self.name} [DetectWorldObjects::setup()]")

    def initialise(self):
        """Initialize robot and client objects for behavior on first tick."""
        self.logger.debug(f"  {self.name} [DetectWorldObjects::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="world_objects", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.world_objects = self.blackboard.perception.world_objects
        self.client = self.robot.ensure_client(WorldObjectClient.default_service_name)

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [DetectWorldObjects::update()]")
        try:
            # List all world objects in the scene.
            self.world_objects = self.client.list_world_objects().world_objects
            # Examine the transform snapshot for the world object!
            for obj in self.world_objects:
                print(f"ID: {obj.id}")
                full_snapshot = obj.transforms_snapshot
                for edge in full_snapshot.child_to_parent_edge_map:
                    print(
                        f"Child frame name: {edge}. Parent frame name: "
                        f"{full_snapshot.child_to_parent_edge_map[edge].parent_frame_name}"
                    )
        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.world_objects = self.world_objects
        self.logger.debug(
            f" {self.name} [DetectWorldObjects::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
