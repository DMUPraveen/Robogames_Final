from typing import Iterable, List, Tuple, Deque, Set, Dict, Optional
from dave_lib import Dave
from Motion_Control_Class import Motion_Control
from enum import Enum
from collections import deque
from Occupancy_grid import Cartesian_to_Grid, Occupancy_Grid, flat_array_to_column_vector
from dda_algo import perform_dda
import numpy as np
from queue import Queue


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
        self.not_visited_count = 0
        self.expected_distance = 0
        self.close_enough_delta = 1
        self.not_visited_threshold = 5

    def close_enough(self, index, other_index):
        return abs(index-other_index) < 1

    def stopping_condition_cheker(self, cell_index_tuple: Tuple[int, int], distance: float):
        row, column = cell_index_tuple[1], cell_index_tuple[0]
        # print(row, column, distance)
        # self.not_visited_count += int(
        #     self.occupancy_grid.grid[row, column] != self.occupancy_grid.CELL_VISITED)
        # print(self.destination, row, column)
        # if(self.occupancy_grid.grid[row, column] != self.occupancy_grid.CELL_VISITED):
        #     return True
        if(self.occupancy_grid.grid[row, column] == self.occupancy_grid.OBSTACLE):
            # print("aborint because of obstacle")
            return True
        if not(0 <= row < self.occupancy_grid.height and 0 <= column < self.occupancy_grid.width):
            # print("aborting because out of depth")
            return True
        if(self.close_enough(row, self.destination[0]) and self.close_enough(column, self.destination[1])):
            # print("abroting because it has been found!")
            return True
        if(distance > 2*self.expected_distance):
            # print(distance, self.expected_distance)
            # print("aborting becuase max_distance exceeded")
            return True
        return False

    def check_conectivity(self, start_x_pos: float, start_y_pos: float, end_x_pos: float, end_y_pos: float, distance_threshold: float):
        self.not_visited_count = 0
        self.destination = self.cart_to_grid(end_x_pos, end_y_pos)
        scaled_start_x_pos, scaled_start_y_pos = self.cart_to_grid.scale_xy_to_grid_scale(
            start_x_pos, start_y_pos)

        scaled_end_x_pos, scaled_end_y_pos = self.cart_to_grid.scale_xy_to_grid_scale(
            end_x_pos, end_y_pos)
        direction = (
            scaled_end_x_pos - scaled_start_x_pos,
            scaled_end_y_pos - scaled_start_y_pos
        )

        # print(scaled_start_x_pos, scaled_start_y_pos)
        # print(scaled_end_x_pos, scaled_end_y_pos)
        # print(f"{direction=}")
        self.expected_distance = np.linalg.norm(direction)
        scaled_starting_point = (scaled_start_x_pos, scaled_start_y_pos)
        return_cell, distance = perform_dda(
            direction,
            scaled_starting_point,
            self.stopping_condition_cheker,
            self.on_visit,
        )
        return_cell = return_cell[1], return_cell[0]
        # print(return_cell, self.destination,
        #       distance_threshold, distance)
        # print(self.not_visited_count)
        if(self.not_visited_threshold > 5):
            print("not visited threshold exceeded")
            return False
        if(abs(self.expected_distance-distance) < self.cart_to_grid.scale_lengths(distance_threshold) or return_cell == self.destination):
            return True
        # print("What happned")
        return False


class TopoNode:
    def __init__(self, index: int, position: Tuple[float, float]):
        self.index = index
        self.connections: Set[TopoNode] = set()
        self.position = position

    def conect_node(self, node: 'TopoNode'):
        if(node in self.connections):
            return
        # print(f"connecting_node {node} to {self}")
        self.connections.add(node)

    def is_connected(self, node: 'TopoNode'):
        return (node in self.connections)

    def __hash__(self) -> int:
        return self.index

# to be removed
    def __str__(self):
        return f"{self.index}->{self.position}"

    def __repr__(self):
        return self.__str__()

    def __eq__(self, __o: 'TopoNode') -> bool:
        return (self.index == __o.index)


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

    def spread_out_in_distance(self, nodes: List[TopoNode], proposed_x: float, proposed_y: float):
        if(len(nodes) == 0):
            return float('inf')
        return min(calculate_distance_between_node_and_position_in_meters(
            node, proposed_x, proposed_y) for node in nodes)

    def add_node(self, row: int, column: int, position_x: float, position_y: float):
        new_node = self.node_maker.make_node((position_x, position_y))
        self.topo_grid[row][column].append(
            new_node
        )
        return new_node

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
            node2.position[1],
            threshold_distance
        )

    def can_connect_position(self, node1: TopoNode, position_x: float, position_y: float, threshold_distance: float):
        return self.reachability_checker.check_conectivity(
            node1.position[0],
            node1.position[1],
            position_x,
            position_y,
            threshold_distance
        )

    def construct_topo_map(self, dave: Dave):
        row, column = self.cartesian_to_grid(dave.x, dave.y)
        nodes = self.get_nodes_in_grid_position(row, column)
        length_in_grid = len(nodes)

        neighbour_topo_cells = self.get_neighbouring_topo_cells(row, column)
        neighbour_topo_cells.append((row, column))

        all_nodes: List[TopoNode] = []

        for neighbour_topo_cell in neighbour_topo_cells:
            all_nodes.extend(
                self.get_nodes_in_grid_position(*neighbour_topo_cell))

        distance_min = self.spread_out_in_distance(
            all_nodes, dave.x, dave.y)

        compatible = (
            length_in_grid < self.max_nodes_per_cell) and self.placement_distance < distance_min
        if compatible:
            new_node = self.add_node(row, column, dave.x, dave.y)
            all_nodes.append(new_node)

        for i in range(len(all_nodes)):
            for j in range(i+1, len(all_nodes)):
                our_node = all_nodes[i]
                other_node = all_nodes[j]
                # print("dave, ", self.reachability_checker.cart_to_grid(
                #     dave.x, dave.y))
                if(our_node.index == other_node.index):
                    continue
                if(our_node.is_connected(other_node)):
                    continue
                if not(self.can_connect(our_node, other_node, self.reachability_distance_threshold)):
                    # print(
                    #     f"{our_node} and {other_node} could not be connected")
                    continue

                our_node.conect_node(other_node)
                other_node.conect_node(our_node)

    def get_all_nodes_in_cells(self, cells: List[Tuple[int, int]]):
        all_nodes: List[TopoNode] = []
        for cell in cells:
            row, column = cell
            all_nodes.extend(self.get_nodes_in_grid_position(row, column))
        return all_nodes

    @staticmethod
    def min_distance_key(node: TopoNode, x_pos: float, y_pos: float):
        return (node.position[0]-x_pos)**2+(node.position[1]-y_pos)**2

    def find_all_close_nodes(self, x_pos, y_pos):
        topo_cell = self.cartesian_to_grid(x_pos, y_pos)
        neighbouring_topo_cells = self.get_neighbouring_topo_cells(*topo_cell)
        neighbouring_topo_cells.append(topo_cell)
        return self.get_all_nodes_in_cells(neighbouring_topo_cells)

    def find_closest_node(self, x_pos: float, y_pos: float):
        all_nodes = self.find_all_close_nodes(x_pos, y_pos)
        if(len(all_nodes) == 0):
            return None
        return min(all_nodes, key=lambda node: self.min_distance_key(node, x_pos, y_pos))


def bfs_find(target_nodes: Iterable[TopoNode], start_node: TopoNode):
    bfs_queue = Queue()
    targets = set(target_nodes)
    bfs_queue.put(start_node)
    path = []
    visited: Set[TopoNode] = set()
    visited.add(start_node)
    parent_node: Dict[TopoNode, Optional[TopoNode]] = {}
    destintation = None
    parent_node[start_node] = None
    if(start_node in target_nodes):
        return [start_node]
    while(not bfs_queue.empty()):
        node: TopoNode = bfs_queue.get()
        for connection in node.connections:
            if(connection in visited):
                continue
            visited.add(connection)
            parent_node[connection] = node
            bfs_queue.put(connection)
            if(node in targets):
                destintation = node
                break
    if(destintation is None):
        return None
    path = []
    path_node = destintation
    while(path_node is not None):
        path.append(path_node)
        path_node = parent_node[path_node]
    return list(reversed(path))


def find_best_path_possible(topo_map: Topological_Map, position_start: Tuple[float, float], position_target: Tuple[float, float]):
    closest_node = topo_map.find_closest_node(*position_start)
    if(closest_node is None):
        return None
    target_nodes = topo_map.find_all_close_nodes(*position_target)
    # print(closest_node, target_nodes)
    path = bfs_find(target_nodes, closest_node)
    return path


def transform_node_list_to_point_follow_list(path_list: List[TopoNode]):
    return [node.position for node in path_list]


class Dashability_Checker:
    def __init__(self, occupancy_grid: Occupancy_Grid, cart_to_grid: Cartesian_to_Grid):
        self.occupancy_grid = occupancy_grid
        self.on_visit = lambda x: None
        self.cart_to_grid = cart_to_grid
        self.destination = (0, 0)
        self.obstalce_detected = False

    def close_enough(self, index, other_index):
        return abs(index-other_index) < 1

    def stopping_condition_cheker(self, cell_index_tuple: Tuple[int, int], distance: float):
        row, column = cell_index_tuple[1], cell_index_tuple[0]
        # print(row, column, distance)
        # self.not_visited_count += int(
        #     self.occupancy_grid.grid[row, column] != self.occupancy_grid.CELL_VISITED)
        # print(self.destination, row, column)
        # if(self.occupancy_grid.grid[row, column] != self.occupancy_grid.CELL_VISITED):
        #     return True
        if(self.occupancy_grid.grid[row, column] == self.occupancy_grid.OBSTACLE):
            self.obstalce_detected = True
            return True
        # if(self.occupancy_grid.grid[row, column] != self.occupancy_grid.CELL_VISITED):
        #     return True
        if not(0 <= row < self.occupancy_grid.height and 0 <= column < self.occupancy_grid.width):
            # print("aborting because out of depth")
            return True
        if(self.close_enough(row, self.destination[0]) and self.close_enough(column, self.destination[1])):
            # print("abroting because it has been found!")
            return True
        if(distance > 2*self.expected_distance):
            # print(distance, self.expected_distance)
            # print("aborting becuase max_distance exceeded")
            return True
        return False

    def check_dashability(self, start_x_pos: float, start_y_pos: float, end_x_pos: float, end_y_pos: float):
        self.obstalce_detected = False
        self.destination = self.cart_to_grid(end_x_pos, end_y_pos)
        scaled_start_x_pos, scaled_start_y_pos = self.cart_to_grid.scale_xy_to_grid_scale(
            start_x_pos, start_y_pos)

        scaled_end_x_pos, scaled_end_y_pos = self.cart_to_grid.scale_xy_to_grid_scale(
            end_x_pos, end_y_pos)
        direction = (
            scaled_end_x_pos - scaled_start_x_pos,
            scaled_end_y_pos - scaled_start_y_pos
        )

        # print(scaled_start_x_pos, scaled_start_y_pos)
        # print(scaled_end_x_pos, scaled_end_y_pos)
        # print(f"{direction=}")
        self.expected_distance = np.linalg.norm(direction)
        scaled_starting_point = (scaled_start_x_pos, scaled_start_y_pos)
        return_cell, distance = perform_dda(
            direction,
            scaled_starting_point,
            self.stopping_condition_cheker,
            self.on_visit,
        )
        if(self.obstalce_detected):
            return False
        return True
