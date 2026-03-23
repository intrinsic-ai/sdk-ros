#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosidl_runtime_py.utilities import get_action
from rosidl_runtime_py.set_message import set_message_fields

from absl import logging
from intrinsic.skills.python import skill_interface
from intrinsic.util.decorators import overrides
from generic_action_skill import generic_action_skill_pb2

class GenericActionSkill(skill_interface.Skill):
    """
    A generic skill to call any ROS 2 Action Server.
    The auto-generated Intrinsic wrapper handles the server; this class handles the logic.
    """

    def __init__(self):
        super().__init__()
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('generic_action_skill_node')
        logging.info("Generic Action Skill Logic Initialized.")

    @overrides(skill_interface.Skill)
    def execute(
        self,
        request: skill_interface.ExecuteRequest,
        context: skill_interface.ExecuteContext,
    ) -> generic_action_skill_pb2.GenericActionSkillResult:
        
        action_name = request.params.action_name
        action_type_str = request.params.action_type
        goal_yaml_str = request.params.goal_yaml
        timeout_sec = request.params.timeout_sec

        logging.info(f"Executing ROS 2 Action: {action_name}")

        try:
            action_class = get_action(action_type_str)
            client = ActionClient(self.node, action_class, action_name)

            if not client.wait_for_server(timeout_sec=timeout_sec):
                raise skill_interface.SkillError(f"Action server {action_name} not found.")

            goal_dict = yaml.safe_load(goal_yaml_str)
            goal_msg = action_class.Goal()
            set_message_fields(goal_msg, goal_dict)

            logging.info("Sending goal...")
            send_goal_future = client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                raise skill_interface.SkillError("Goal rejected by server.")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)

            logging.info(f"Action finished with status: {result_future.result().status}")

        except Exception as e:
            logging.error(f"Execution failed: {e}")
            raise skill_interface.SkillError(str(e))

        return generic_action_skill_pb2.GenericActionSkillResult()