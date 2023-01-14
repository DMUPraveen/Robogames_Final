from typing import Iterable, List, Tuple, Deque, Set
from dave_lib import Dave
from Motion_Control_Class import Motion_Control
from enum import Enum
from collections import deque
from Occupancy_grid import Cartesian_to_Grid


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
        self.motion_controller.set_target(*self.path_points.popleft())

    def run(self, dave: Dave,):

        if(self.state != Point_Follow_States.FOLLOWING):
            return
        if not(self.motion_controller.go_to_position(dave, self.first_turn_threshold, self.linear_threshold)):
            return
        if(len(self.path_points) == 0):
            self.state = Point_Follow_States.FINISHED
            return
        self.motion_controller.set_target(*self.path_points.popleft())


class TopoNode:
    def __init__(self, index: int, position: Tuple[float, float]):
        self.index = index
        self.connections: Set[TopoNode] = set()
        self.position = position

    def __hash__(self) -> int:
        return self.index


class Node_Maker:
    def __init__(self):
        self.index = -1

    def make_node(self, position: Tuple[float, float]):
        self.index += 1
        return TopoNode(self.index, position)


class Topological_Map:
    SAFETY_FACTOR = 10

    def __init__(self, x_width, y_width, scale):
        self.node_maker: Node_Maker = Node_Maker()
        grid_width = int(x_width/scale+0.5)+self.SAFETY_FACTOR
        grid_height = int(y_width/scale+0.5)+self.SAFETY_FACTOR
        width = max(grid_height, grid_width)
        origin_pos = width//2

        self.cartesian_to_grid = Cartesian_to_Grid(
            scale,
            origin_pos,
            origin_pos
        )
        self.topo_grid: List[List[List[TopoNode]]] = [
            [[] for _ in range(width)] for _ in range(width)
        ]

    def construct_topo_map(self, dave: Dave):
        pass
