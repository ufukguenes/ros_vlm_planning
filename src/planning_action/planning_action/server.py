import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from planning_action_interface.action import Planning

import os
import time
from openai import OpenAI
from dotenv import load_dotenv, dotenv_values
from moveit.planning import MoveItPy

from rclpy.executors import MultiThreadedExecutor
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory

from .ActionWrapper.AbstractActionWrapper import AbstractActionWrapper
from .ActionWrapper.DummyActionWrapper import DummyActionWrapper


class PlanningActionServer(Node):

    def __init__(self, action_wrapper: AbstractActionWrapper):
        super().__init__("planning_action_server")

        self.action_wrapper = action_wrapper


        self.get_logger().info("MoveItPy initialized and planning component loaded.")

        self.__action_server = ActionServer(
            self, Planning, "planning", self.execute_callback
        )

        load_dotenv()

        api_key = os.getenv("GOOGLE_AI_KEY")
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

        self.get_logger().info("PlanningActionServer initialized and ready.")

    def execute_callback(self, goal_handle: ServerGoalHandle):

        self.get_logger().info("prompting vlm...")
        prompt = goal_handle.request.prompt

        feedback_msg = Planning.Feedback()
        feedback_msg.incomplete_plan = f"waiting for vlm"
        goal_handle.publish_feedback(feedback_msg)

        response = self.client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[
                {"role": "system", "content": 
                 "You are a robot. "
                 "Pick one or multiple of the following actions which best solves the task."
                 "If there are parameters available, provide parameters for the action."
                 "Parameters are provided as a comma seperated list after each possible action. "
                 "Do not wrap the actions or parameters in additional apostrophes, provide the combination as: action param_1 param_2 ...\n"
                 "Each action should be in its own line"
                 f"Available actions: \n {self.action_wrapper.list_available_parameterizable_actions()}"},
                {"role": "user", "content": prompt},
            ],
        )

        
        feedback_msg.incomplete_plan = f"received message from vlm, executing plan"
        goal_handle.publish_feedback(feedback_msg)

        result = Planning.Result()
        result.plan = response.choices[0].message.content
        print(response.choices[0].message)

        for line in result.plan.split("\n"):
            action, params = self.action_wrapper.convert_string_line_to_action(line)
            print(action, params)
            self.action_wrapper.call_parameterized_action(action, params)

        
        goal_handle.succeed()

        self.get_logger().info("Completed")
        return result


def main(args=None):
    rclpy.init(args=args)

    moveit_config_builder = MoveItConfigsBuilder("panda")
    moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("panda_moveit_config") + "/config/moveit_cpp.yaml") 
    moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()
    moveit_node = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)

    action_wrapper = DummyActionWrapper(moveit_node)
    planning_action_server = PlanningActionServer(action_wrapper)

    executor = MultiThreadedExecutor()

    executor.add_node(planning_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        planning_action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
