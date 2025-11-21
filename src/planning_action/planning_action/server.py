import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from planning_action_interface.action import Planning

import os
import time
from openai import OpenAI
from dotenv import load_dotenv, dotenv_values
from rclpy.action import ActionClient
from moveit.planning import PlanningComponent, MoveItPy

from rclpy.executors import MultiThreadedExecutor
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


class PlanningActionServer(Node):

    def __init__(self, moveit_node):
        super().__init__("planning_action_server")

        self.moveit_node = moveit_node

        self.planning_component = self.moveit_node.get_planning_component("panda_arm")
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
        self.get_logger().info("Creating plan...")

        self.planning_component.set_start_state(configuration_name="ready")
        self.planning_component.set_goal_state(configuration_name="extended")

        plan_result = self.planning_component.plan()
        self.moveit_node.execute(plan_result.trajectory, blocking=True)

        prompt = goal_handle.request.prompt

        """response = self.client.chat.completions.create(
            model="gemini-2.5-flash",
            messages=[
                {"role": "system", "content": "You are a robot."},
                {"role": "user", "content": prompt},
            ],
        )

        feedback_msg = Planning.Feedback()
        for i in range(10):
            feedback_msg.incomplete_plan = f"waiting for vlm: {i}"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.01)"""

        result = Planning.Result()
        # result.plan = response.choices[0].message.content
        goal_handle.succeed()

        # print(response.choices[0].message)

        self.get_logger().info("Completed")
        return result


def main(args=None):
    rclpy.init(args=args)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda", package_name="panda_moveit_config")
        .robot_description(file_path="config/panda.urdf.xacro")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
        .to_dict()
    )

    moveit_py_node = MoveItPy(node_name="moveit_py", config_dict=moveit_config)

    planning_action_server = PlanningActionServer(moveit_py_node)

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
