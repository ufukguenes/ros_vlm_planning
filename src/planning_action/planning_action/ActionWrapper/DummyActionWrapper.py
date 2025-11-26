from .AbstractActionWrapper import AbstractActionWrapper
from enum import Enum

from moveit.planning import PlanningComponent
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class DummyActions(Enum):
    GRIPPER = ("open_or_close", "open,close")
    ARM_READY_POSE = ("ready",)
    RANDOM_POSE = ("random_pose",)
    POINT = ("point", "x_pos", "y_pos")
    MOVE_ABOVE = ("move_above", "x_pos", "y_pos", "z_pos")

    @classmethod
    def from_value(cls, action_value: str):
        for entry in cls:
            if entry.value[0] == action_value:
                return entry

    @classmethod
    def to_list(cls):
        all_actions_str = ""
        for action in cls:
            all_actions_str += "".join(action.value) + " \n"

        return all_actions_str



class DummyActionWrapper(AbstractActionWrapper):

    def __init__(self, moveit_node):
        super().__init__(moveit_node)

        
    def list_available_parameterizable_actions(self) -> str:
        return DummyActions.to_list()
    

    def call_parameterized_action(self, action: str, parameters: list[str], header: Header) -> None:
        print(f"dummy execution of {action} with parameters {parameters}")

        print(action)
        action_member = DummyActions.from_value(action)
        print(action_member)
        match action_member:
            case DummyActions.GRIPPER:
                self.set_action(self.hand, configuration_name=parameters[0])

            case DummyActions.ARM_READY_POSE:
                self.set_action(self.panda_arm, configuration_name="ready")

            case DummyActions.RANDOM_POSE:
                self.robot_state.set_to_random_positions()
                self.set_action(self.panda_arm, robot_state=self.robot_state)

            case DummyActions.POINT:
                print(action, parameters)

            case DummyActions.MOVE_ABOVE:
                pose_stamped = self.create_pose_from_xyz(*parameters, header)
                self.set_action(self.panda_arm, pose_stamped_msg=pose_stamped, pose_link='panda_hand')

            case _:
                raise ValueError(f"Unknown action string: {action}")
    
    def convert_string_line_to_action(self, line: str) -> tuple[str, list[str]]:
        elements = line.split(" ")
        return elements[0], elements[1:]

    def set_action(self, planning_component: PlanningComponent, **kwargs):
        planning_component.set_start_state_to_current_state()
        print(kwargs)
        planning_component.set_goal_state(**kwargs)

        plan_result = planning_component.plan()
        self.moveit_node.execute(plan_result.trajectory, controllers=[])


    def create_pose_from_xyz(self, x, y, z, header: Header, frame_id="world"):
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.header.frame_id = frame_id

        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        return pose_stamped