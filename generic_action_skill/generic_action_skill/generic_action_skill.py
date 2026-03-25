import yaml
import sys
import rclpy
import pkgutil
import importlib
import time
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py.set_message import set_message_fields

from absl import logging, app, flags
from intrinsic.skills.python import skill_interface
from intrinsic.util.decorators import overrides
from generic_action_skill import generic_action_skill_pb2


# To solve unkwon flag errors form Flowstate. 
FLAGS = flags.FLAGS

# Primary Platform Flags
flags.DEFINE_integer('port', 8003, 'Port to listen on.', allow_override=True)
flags.DEFINE_string('skill_service_config_filename', '', 'Path to config.', allow_override=True)
flags.DEFINE_boolean('logtostderr', False, 'Log to stderr.', allow_override=True)
flags.DEFINE_boolean('alsologtostderr', False, 'Log to stderr and files.', allow_override=True)
flags.DEFINE_integer('v', 0, 'Verbosity level.', allow_override=True)
flags.DEFINE_string('log_dir', '/tmp', 'Directory for logs.', allow_override=True)
flags.DEFINE_integer('grpc_connect_timeout_secs', 60, 'gRPC connect timeout.', allow_override=True)

# Platform Service Addresses
flags.DEFINE_string('data_logger_grpc_service_address', '', 'Data logger address.', allow_override=True)
flags.DEFINE_string('world_service_address', '', 'World service address.', allow_override=True)
flags.DEFINE_string('geometry_service_address', '', 'Geometry service address.', allow_override=True)
flags.DEFINE_string('motion_planner_service_address', '', 'Motion planner address.', allow_override=True)
flags.DEFINE_string('skill_registry_service_address', '', 'Skill registry address.', allow_override=True)

# Telemetry and Monitoring
flags.DEFINE_boolean('opencensus_tracing', False, 'Enable OpenCensus tracing.', allow_override=True)
flags.DEFINE_float('opencensus_sampling_rate', 1.0, 'Sampling rate for tracing.', allow_override=True)
flags.DEFINE_integer('opencensus_metrics_port', 0, 'Port for OpenCensus metrics.', allow_override=True)
flags.DEFINE_string('opencensus_address', '', 'Address for OpenCensus exporter.', allow_override=True)
flags.DEFINE_string('statsd_address', '', 'Address for StatsD exporter.', allow_override=True)
flags.DEFINE_integer('statsd_port', 0, 'Port for StatsD exporter.', allow_override=True)

_GLOBAL_SERVER_HANDLE = None

class GenericActionSkill(skill_interface.Skill):
    """
    A generic skill to call any ROS 2 Action Server.
    Initialization of ROS is lazy to ensure the gRPC server starts.
    """

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.node = None
        logging.info("GenericActionSkill created. ROS initialization will be lazy.")

    def _ensure_ros_initialized(self):
        """Ensures ROS 2 is initialized and the node is created."""
        if self.node is not None:
            return

        logging.info("Initializing ROS 2 and creating node...")
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('generic_action_skill_node')
        logging.info("ROS 2 Node 'generic_action_skill_node' is ready.")

    @overrides(skill_interface.Skill)
    def execute(
        self,
        request: skill_interface.ExecuteRequest,
        context: skill_interface.ExecuteContext,
    ) -> generic_action_skill_pb2.GenericActionSkillResult:
        
        self._ensure_ros_initialized()

        action_name = request.params.action_name
        action_type_str = request.params.action_type
        goal_yaml_str = request.params.goal_yaml
        timeout_sec = request.params.timeout_sec

        logging.info(f"Executing ROS 2 Action: {action_name} of type {action_type_str}")

        try:
            action_class = get_action(action_type_str)
            client = ActionClient(self.node, action_class, action_name)

            if not client.wait_for_server(timeout_sec=timeout_sec):
                raise skill_interface.SkillError(f"Action server {action_name} not found after {timeout_sec}s.")

            goal_dict = yaml.safe_load(goal_yaml_str)
            goal_msg = action_class.Goal()
            set_message_fields(goal_msg, goal_dict)

            send_goal_future = client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                raise skill_interface.SkillError("Goal was rejected by the ROS 2 action server.")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)

            logging.info(f"Action finished with status: {result_future.result().status}")

        except Exception as e:
            logging.error(f"Generic Action Skill execution failed: {e}")
            raise skill_interface.SkillError(str(e))

        return generic_action_skill_pb2.GenericActionSkillResult()

def main(argv):
    """
    Entry point for the skill service.
    """
    global _GLOBAL_SERVER_HANDLE
    logging.info(f"Starting skill service on port: {FLAGS.port}")
    
    try:
        from intrinsic.skills.generator import app as generator_app
        logging.info("Starting Intrinsic generator-based runner...")
        _GLOBAL_SERVER_HANDLE = generator_app.run(GenericActionSkill)
        logging.info(f"Skill runner initialized. Handle type: {type(_GLOBAL_SERVER_HANDLE)}")

        if _GLOBAL_SERVER_HANDLE is not None:
            if hasattr(_GLOBAL_SERVER_HANDLE, 'wait_for_termination'):
                _GLOBAL_SERVER_HANDLE.wait_for_termination()
        return 0
    except (ImportError, AttributeError) as e:
        logging.error(f"Generator runner failed: {e}")
        try:
            from intrinsic.skills.internal import single_skill_factory
            logging.info("Falling back to single_skill_factory...")
            _GLOBAL_SERVER_HANDLE = single_skill_factory.run(GenericActionSkill)
            return 0
        except Exception as e2:
            logging.error(f"Fallback failed: {e2}")

    raise RuntimeError("Could not successfully start any Intrinsic Skill Runner.")

if __name__ == '__main__':
    def parse_flags(argv):
        return FLAGS(argv, known_only=True)
    
    try:
        # Run the main application
        app.run(main, flags_parser=parse_flags)
    except SystemExit:
        # Catch sys.exit calls from within absl.app.run to prevent process termination.
        # This keeps the background gRPC server alive.
        logging.info("Service initialized. Hard-blocking process to keep gRPC port 8003 active.")
    except KeyboardInterrupt:
        logging.info("Skill service stopped by user.")
        sys.exit(0)
    while True:
        time.sleep(3600)