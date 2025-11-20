import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from planning_action_interface.action import Planning

import os
import time
from openai import OpenAI
from dotenv import load_dotenv, dotenv_values


class PlanningActionServer(Node):

    def __init__(self):
        super().__init__("planning_action_server")
        self.__action_server = ActionServer(
            self, Planning, "planning", self.execute_callback
        )

        load_dotenv()

        api_key = os.getenv("GOOGLE_AI_KEY")
        self.client = OpenAI(
            api_key=api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Creating plan...")

        prompt = goal_handle.request.prompt

        response = self.client.chat.completions.create(
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
            time.sleep(0.01)

        result = Planning.Result()
        result.plan = response.choices[0].message.content
        goal_handle.succeed()

        print(response.choices[0].message)

        self.get_logger().info("Completed")
        return result


def main(args=None):
    rclpy.init(args=args)
    planning_action_server = PlanningActionServer()
    rclpy.spin(planning_action_server)


if __name__ == "__main__":
    main()
