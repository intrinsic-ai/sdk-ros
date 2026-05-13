import yaml
import sys
import importlib
import grpc
from datetime import timedelta
from concurrent import futures
from typing import Any

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_yaml

# Intrinsic / ABSL Imports
from absl import logging, app, flags
from intrinsic.skills.python import skill_interface
from intrinsic.util.decorators import overrides
from generic_action_skill import generic_action_skill_pb2

FLAGS = flags.FLAGS
flags.DEFINE_integer("port", 8003, "Port to listen on.", allow_override=True)
flags.DEFINE_string(
    "skill_service_config_filename", "", "Path to config.", allow_override=True
)
flags.DEFINE_string(
    "world_service_address", "", "World service address.", allow_override=True
)
flags.DEFINE_string(
    "geometry_service_address", "", "Geometry service address.", allow_override=True
)
flags.DEFINE_string(
    "motion_planner_service_address", "", "Motion planner address.", allow_override=True
)


class GenericActionSkill(skill_interface.Skill):
    """
    A generic skill to call any ROS 2 Action Server and return results to Flowstate.
    """

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.node = None
        self._skill_proto = None
        logging.info("GenericActionSkill instance initialized.")

    def _ensure_ros(self):
        """Lazy-initialization of the ROS 2 node."""
        if self.node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = Node("generic_action_skill_node")
            logging.info("ROS 2 Node ready.")

    def get_skill_runtime_data(self, skill_name: str):
        """
        Dynamically satisfies SDK metadata requirements.
        Uses an internal helper class to satisfy platform probes (Full_name, Descriptor, etc.)
        """

        class SmartContainer:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)

            def __getattr__(self, name):
                lower = name.lower()
                if "timeout" in lower:
                    return timedelta(seconds=180)
                if "ready" in lower:
                    return timedelta(seconds=60)
                if "supports" in lower:
                    return True
                if any(x in lower for x in ["topics", "resources", "specs"]):
                    return [] if "topics" in lower else {}
                return SmartContainer()

        # Provide descriptors from generated pb2 so Flowstate can map UI inputs
        params = SmartContainer(
            descriptor=generic_action_skill_pb2.GenericActionSkillParams.DESCRIPTOR,
            default_value=None,
        )
        returns = SmartContainer(
            message_full_name=generic_action_skill_pb2.GenericActionSkillResult.DESCRIPTOR.full_name
        )

        return SmartContainer(
            skill=self._skill_proto,
            skill_id="com.example.generic_action_skill",
            parameter_data=params,
            return_type_data=returns,
            execution_options=SmartContainer(),
            topic_data=SmartContainer(),
            status_specs=SmartContainer(),
            resource_data=SmartContainer(),
        )

    def get_skill_execute(self, name):
        return self

    def get_skill_project(self, name):
        return self

    def predict(self, request: Any, context: Any) -> Any:
        """Probes the container for action type availability."""
        params = getattr(request, "params", request)
        action_type = getattr(params, "action_type", "")
        try:
            pb2 = importlib.import_module("intrinsic.skills.proto.skill_service_pb2")
            result = pb2.PredictResult()
            if action_type:
                get_action(action_type)
            return result
        except Exception:
            logging.warning(f"Action type '{action_type}' not available.")
            return pb2.PredictResult() if "pb2" in locals() else None

    @overrides(skill_interface.Skill)
    def execute(
        self, request, context
    ) -> generic_action_skill_pb2.GenericActionSkillResult:
        self._ensure_ros()
        p = request.params
        logging.info(f"Goal: {p.action_name} [{p.action_type}]")

        skill_result = generic_action_skill_pb2.GenericActionSkillResult()

        try:
            # 1. Load Action and Create Client
            action_class = get_action(p.action_type)
            client = ActionClient(self.node, action_class, p.action_name)

            if not client.wait_for_server(timeout_sec=p.timeout_sec):
                raise skill_interface.SkillError(
                    2, f"Action server {p.action_name} timed out."
                )

            # 2. Build Goal from YAML string
            goal_msg = action_class.Goal()
            set_message_fields(goal_msg, yaml.safe_load(p.goal_yaml))

            # 3. Send Goal
            future = client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, future)

            handle = future.result()
            if not handle.accepted:
                skill_result.status = "REJECTED"
                return skill_result

            # 4. Wait for Result
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, res_future)

            # 5. Extract Result and Payload
            action_res = res_future.result()
            status_map = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}

            skill_result.status = status_map.get(
                action_res.status, f"UNKNOWN({action_res.status})"
            )
            skill_result.result_yaml = message_to_yaml(action_res.result)

            logging.info(f"Finished: {skill_result.status}")

        except Exception as e:
            logging.error(f"Execution Error: {e}")
            raise skill_interface.SkillError(4, str(e))

        return skill_result


def start_runner(argv):
    """
    Manual gRPC runner. This is necessary because standard SDK app.run
    can fail in specific ROS 2 Jazzy cluster environments.
    """
    from intrinsic.skills.internal import skill_service_impl

    # Helper to find pb2 modules in various SDK namespaces
    def find_mod(name):
        for p in [
            "intrinsic.skills.v1",
            "intrinsic.skills.proto",
            "intrinsic.proto.skills",
        ]:
            try:
                return importlib.import_module(f"{p}.{name}")
            except:
                continue
        return None

    skills_pb2 = find_mod("skills_pb2")
    skill_proto = skills_pb2.Skill() if skills_pb2 else None

    # Helper to wrap Platform Service Stubs (World, Motion, Geometry)
    def wrap_stub(patterns, addr):
        if not addr:
            return None
        for p in [
            "intrinsic.world.v1",
            "intrinsic.motion_planning.public.proto.v1",
            "intrinsic.geometry.v1",
        ]:
            try:
                mod = importlib.import_module(f"{p}.{patterns[0]}_pb2_grpc")
                return getattr(mod, f"{patterns[1]}Stub")(grpc.insecure_channel(addr))
            except:
                continue
        return None

    world = wrap_stub(
        ["object_world_service", "ObjectWorldService"], FLAGS.world_service_address
    )
    motion = wrap_stub(
        ["motion_planner_service", "MotionPlannerService"],
        FLAGS.motion_planner_service_address,
    )
    geom = wrap_stub(
        ["geometry_service", "GeometryService"], FLAGS.geometry_service_address
    )

    skill_instance = GenericActionSkill()
    skill_instance._skill_proto = skill_proto
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

    class UnifiedServicerProxy(object):
        """Bridges gRPC requests to the internal servicers."""

        def __init__(self, inst, proto, world, motion, geom):
            self._skill = inst
            try:
                self._executor = skill_service_impl.SkillExecutorServicer(
                    inst, world, motion, geom
                )
            except TypeError:
                self._executor = skill_service_impl.SkillExecutorServicer(
                    inst, world, motion, geom, None
                )
            self._info = skill_service_impl.SkillInformationServicer(proto)

        def Predict(self, r, c):
            return self._skill.predict(r, c)

        def __getattr__(self, name):
            # Fallback to executor or info servicers for standard Intrinsic calls
            for target in [self._executor, self._info]:
                if hasattr(target, name):
                    return getattr(target, name)
            return lambda r, c: self._skill.predict(r, c)

    proxy = UnifiedServicerProxy(skill_instance, skill_proto, world, motion, geom)

    # Register all gRPC interfaces (Discovery loop for v1 vs proto namespaces)
    mods = ["skill_service", "skill_executor", "skill_information", "skill"]
    for m in mods:
        for p in ["intrinsic.skills.v1", "intrinsic.skills.proto"]:
            try:
                mod = importlib.import_module(f"{p}.{m}_pb2_grpc")
                for attr in dir(mod):
                    if "add_" in attr and "_to_server" in attr:
                        getattr(mod, attr)(proxy, server)
            except:
                continue

    server.add_insecure_port(f"[::]:{FLAGS.port}")
    server.start()
    logging.info(f"gRPC Server listening on port {FLAGS.port}")
    server.wait_for_termination()


if __name__ == "__main__":
    try:
        app.run(start_runner, flags_parser=lambda argv: FLAGS(argv, known_only=True))
    except (SystemExit, KeyboardInterrupt):
        sys.exit(0)
