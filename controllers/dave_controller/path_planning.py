from typing import Iterable, List, Tuple, Deque, Set
from dave_lib import Dave
from Motion_Control_Class import Motion_Control
from enum import Enum
from collections import deque
from Occupancy_grid import Cartesian_to_Grid, Occupancy_Grid, flat_array_to_column_vector
from dda_algo import perform_dda
import numpy as np


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


class Reachability_Checker:
    def __init__(self, occupancy_grid: Occupancy_Grid, cart_to_grid: Cartesian_to_Grid):
        self.occupancy_grid = occupancy_grid
        self.on_visit = lambda x: None
        self.cart_to_grid = cart_to_grid
        self.destination = (0, 0)

    def stopping_condition_cheker(self, cell_index_tuple: Tuple[int, int], distance: float):
        row, column = cell_index_tuple[1], cell_index_tuple[0]
        if(self.occupancy_grid.grid[row, column] != self.occupancy_grid.CELL_VISITED):
            return True
        if(0 <= row < self.occupancy_grid.height and 0 <= column < self.occupancy_grid.width):
            return True
        if(row == self.destination[0] and column == self.destination[1]):
            return True
        return False

    def check_conectivity(self, start_x_pos: float, start_y_pos: float, end_x_pos: float, end_y_pos: float, distance_threshold: float):
        self.destination = self.cart_to_grid(end_x_pos, end_y_pos)
        scaled_start_x_pos, scaled_start_y_pos = self.cart_to_grid.scale_xy_to_grid_scale(
            start_x_pos, start_y_pos)

        scaled_end_x_pos, scaled_end_y_pos = self.cart_to_grid.scale_xy_to_grid_scale(
            end_x_pos, end_y_pos)
        direction = (
            scaled_end_x_pos - scaled_start_x_pos,
            scaled_end_y_pos - scaled_start_y_pos
        )
        expected_distance = np.linalg.norm(direction)
        scaled_starting_point = (scaled_start_x_pos, scaled_start_y_pos)
        return_cell, distance = perform_dda(
            direction,
            scaled_starting_point,
            self.stopping_condition_cheker,
            self.on_visit,
        )
        if(abs(distance-expected_distance) < distance_threshold):
            return True
        return False


class TopoNode:
    def __init__(self, index: int, position: Tuple[float, float]):
        self.index = index
        self.connections: Set[TopoNode] = set()
        self.position = position

    def conect_node(self, node: 'TopoNode'):
        if(node in self.connections):
            return
        self.connections.add(node)

    def __hash__(self) -> int:
        return self.index


def calculate_distance_between_nodes_in_meters(node1: TopoNode, node2: TopoNode):
    return float(np.linalg.norm(
        np.array(node1.position)-np.array(node2.position)))  # type: ignore


def calculate_distance_between_node_and_position_in_meters(node1: TopoNode, position_x: float, position_y: float):

    return float(np.linalg.norm(
        np.array(node1.position)-np.array((position_x, position_y))))  # type: ignore


class Node_Maker:
    def __init__(self):
        self.index = -1

    def make_node(self, position: Tuple[float, float]):
        self.index += 1
        return TopoNode(self.index, position)


class Topological_Map:
    SAFETY_FACTOR = 10
    NEIGHBOUR_DIRECTIONS: List[Tuple[int, int]] = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, -1),
        (1, 1),
        (-1, 1),
        (-1, -1)
    ]

    def __init__(self, x_width: float, y_width: float, scale: float,
                 placement_width: float, max_nodes_per_cell: int, reachability_checker: Reachability_Checker,
                 reachability_distance_threshold: float):
        self.node_maker: Node_Maker = Node_Maker()
        grid_width = int(x_width/scale+0.5)+self.SAFETY_FACTOR
        grid_height = int(y_width/scale+0.5)+self.SAFETY_FACTOR
        self.width = max(grid_height, grid_width)
        origin_pos = self.width//2
        # how far apart the node are placed in a given cell
        self.placement_distance = placement_width
        self.max_nodes_per_cell = max_nodes_per_cell

        self.cartesian_to_grid = Cartesian_to_Grid(
            scale,
            origin_pos,
            origin_pos
        )
        self.topo_grid: List[List[List[TopoNode]]] = [
            [[] for _ in range(self.width)] for _ in range(self.width)
        ]
        self.reachability_checker = reachability_checker
        self.reachability_distance_threshold = reachability_distance_threshold

    def within_bounds(self, row: int, column: int):
        return (0 <= row < self.width and 0 <= column < self.width)

    def get_nodes_in_grid_position(self, row: int, column: int):
        if(self.within_bounds(row, column)):
            return self.topo_grid[row][column]
        return []

    def spread_out_in_distance(self, nodes: Iterable[TopoNode], proposed_x: float, proposed_y: float):
        return min(calculate_distance_between_node_and_position_in_meters(
            node, proposed_x, proposed_y) for node in nodes)

    def add_node(self, row: int, column: int, position_x: float, position_y: float):
        new_node = self.node_maker.make_node((position_x, position_y))
        self.topo_grid[row][column].append(
            new_node
        )

    def get_neighbouring_topo_cells(self, row: int, column: int):
        neighbours: List[Tuple[int, int]] = []
        for delta_row, delta_column in self.NEIGHBOUR_DIRECTIONS:
            new_row, new_column = row+delta_row, column+delta_column
            if(self.within_bounds(new_row, new_column)):
                neighbours.append((new_row, new_column))
        return neighbours

    def can_connect(self, node1: TopoNode, node2: TopoNode, threshold_distance: float):
        if(node1.index == node2.index):
            return False
        return self.reachability_checker.check_conectivity(
            node1.position[0],
            node1.position[1],
            node2.position[0],
            node1.position[1],
            threshold_distance
        )

    def construct_topo_map(self, dave: Dave):
        row, column = self.cartesian_to_grid(dave.x, dave.y)
        nodes = self.get_nodes_in_grid_position(row, column)

        # adding a new node
        if(len(nodes) < self.max_nodes_per_cell and self.spread_out_in_distance(nodes, dave.x, dave.y) > self.placement_distance):
            added_new_node = self.add_node(row, column, dave.x, dave.y)

        # trying to make new connections
        neighbour_topo_cells = self.get_neighbouring_topo_cells(row, column)
        neighbour_topo_cells.append((row, column))

        for row, column in neighbour_topo_cells:
            other_nodes = self.get_nodes_in_grid_position(row, column)
            for our_node in nodes:
                for other_node in other_nodes:
                    if(our_node.index == other_node.index):
                        continue
                    if not(self.can_connect(our_node, other_node, self.reachability_distance_threshold)):
                        continue
                    our_node.conect_node(other_node)
                    other_node.conect_node(our_node)
