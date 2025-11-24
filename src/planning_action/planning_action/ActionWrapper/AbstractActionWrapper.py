from abc import ABC, abstractmethod

from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState


class AbstractActionWrapper(ABC):

    def __init__(self, moveit_node: MoveItPy):
        super().__init__()
        self.moveit_node = moveit_node
        self.panda_arm: PlanningComponent = self.moveit_node.get_planning_component("panda_arm")
        self.hand: PlanningComponent = self.moveit_node.get_planning_component("hand")
        self.robot_model = self.moveit_node.get_robot_model()
        self.robot_state = RobotState(self.robot_model)


    @abstractmethod
    def list_available_parameterizable_actions(self) -> str: ...

    @abstractmethod
    def call_parameterized_action(self, action: str, parameters: list[str]) -> None: ...

    @abstractmethod
    def convert_string_line_to_action(self, line: str) -> tuple[str, list[str]]: ...


        
