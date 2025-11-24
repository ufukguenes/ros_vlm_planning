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
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory


class PlanningActionServer(Node):

    def __init__(self, moveit_node: MoveItPy):
        super().__init__("planning_action_server")

        self.moveit_node = moveit_node

        self.panda_arm: PlanningComponent = self.moveit_node.get_planning_component("panda_arm")
        self.robot_model = self.moveit_node.get_robot_model()
        self.robot_state = RobotState(self.robot_model)


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

        self.get_logger().info("setting start state...")
        self.panda_arm.set_start_state_to_current_state()
        
        self.get_logger().info("Setting goal state...")
        self.robot_state.set_to_random_positions()
        self.panda_arm.set_goal_state(robot_state=self.robot_state)

        self.get_logger().info("Creating plan...")
        plan_result = self.panda_arm.plan()

        self.get_logger().info("executing plan...")
        self.moveit_node.execute(plan_result.trajectory, controllers=[])

        

        self.get_logger().info("prompting vlm...")
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

    moveit_config_builder = MoveItConfigsBuilder("panda")
    moveit_config_builder.moveit_cpp(file_path=get_package_share_directory("panda_moveit_config") + "/config/moveit_cpp.yaml") 
    moveit_config_dict = moveit_config_builder.to_moveit_configs().to_dict()
    moveit_py_node = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)

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
