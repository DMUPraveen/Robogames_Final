from typing import Iterable, List, Tuple, Deque
from dave_lib import Dave
from Motion_Control_Class import Motion_Control
from enum import Enum
from collections import deque


class Point_Follow_States(Enum):
    IDLE = 0
    FOLLOWING = 1
    FINISHED = 2


class Point_Follow:
    def __init__(self, motion_controller: Motion_Control, first_turn_threshold: float, linear_threshold: float):
        self.motion_controller = motion_controller
        self.state: Point_Follow_States = Point_Follow_States.IDLE
        self.path_points: Deque[Tuple[float, float]] = deque()
        self.first_turn_threshold = first_turn_threshold
        self.linear_threshold = linear_threshold

    def terminate_current_run_and_set_path(self, point_list: Iterable[Tuple[float, float]]):
        self.state = Point_Follow_States.IDLE
        self.path_points = deque(point_list)

    def start_current_path(self):
        if(len(self.path_points) == 0):
            return
        self.state = Point_Follow_States.FOLLOWING
        self.motion_controller.set_target(*self.path_points[0])

    def run(self, dave: Dave,):
        if(self.state != Point_Follow_States.FOLLOWING):
            return
        if(len(self.path_points) == 0):
            self.state = Point_Follow_States.FINISHED
        if not(self.motion_controller.go_to_position(dave, self.first_turn_threshold, self.linear_threshold)):
            return
        self.path_points.popleft()
