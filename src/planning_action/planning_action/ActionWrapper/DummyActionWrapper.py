from .AbstractActionWrapper import AbstractActionWrapper
from enum import Enum

from moveit.planning import PlanningComponent

class DummyActions(Enum):
    GRIPPER = ("open_or_close", "open,close")
    ARM_READY_POSE = ("ready",)
    RANDOM_POSE = ("random_pose",)
    POINT = ("point", "x_pos", "y_pos")

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
    

    def call_parameterized_action(self, action: str, parameters: list[str]) -> None:
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

            case _:
                raise ValueError(f"Unknown action string: {action}")
    
    def convert_string_line_to_action(self, line: str) -> tuple[str, list[str]]:
        elements = line.split(" ")
        return elements[0], elements[1:]

    def set_action(self, planning_component: PlanningComponent, **kwargs):
        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(**kwargs)

        plan_result = planning_component.plan()
        self.moveit_node.execute(plan_result.trajectory, controllers=[])