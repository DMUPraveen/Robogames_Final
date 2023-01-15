from typing import Iterable, List, Tuple, Deque, Set, Dict, Optional
from dave_lib import Dave, update_dave_pose, update_environment, Environment
from Occupancy_grid import Occupancy_Grid, Cartesian_to_Grid, Mapper, get_true_distance_with_maximum_free_distance
from Motion_Control_Class import Motion_Control
from path_planning import Point_Follow, Point_Follow_States, Topological_Map, Reachability_Checker, find_best_path_possible, transform_node_list_to_point_follow_list, Dashability_Checker
from wall_following import attempt2_left_wall_following, attempt2_right_wall_following, any_wall_detected
from enum import Enum
import numpy as np


class Target_Reacher_State(Enum):
    IDLE = 0
    FIND_PATH = 1
    PATH_FOLLOWING = 2
    DASHING = 3
    LEFT_WALL_FOLLOWING = 4
    RIGHT_WALL_FOLLOWING = 5
    FINISHED = 6


class Target_Reacher:
    def __init__(self, occupancy_grid: Occupancy_Grid, dashability_checker: Dashability_Checker,
                 topo_map: Topological_Map, dave: Dave, motion_controller: Motion_Control, point_follow: Point_Follow):
        self.state: Target_Reacher_State = Target_Reacher_State.IDLE
        self.path_to_follow: Optional[List[Tuple[float, float]]] = None
        self.topo_map = topo_map
        self.occupancy_grid = occupancy_grid
        self.dave = dave
        self.target: Optional[Tuple[float, float]] = None
        self.motion_controller = motion_controller
        self.point_follow = point_follow
        self.dashability_checker = dashability_checker

        self.funs_to_run = {
            Target_Reacher_State.IDLE: self.on_idle,
            Target_Reacher_State.DASHING: self.dashing,
            Target_Reacher_State.FIND_PATH: self.on_find_path,
            Target_Reacher_State.PATH_FOLLOWING: self.path_following,
            Target_Reacher_State.LEFT_WALL_FOLLOWING: self.left_wall_following,
            Target_Reacher_State.RIGHT_WALL_FOLLOWING: self.right_wall_following,
        }

    def reset(self):
        if(self.path_to_follow is not None):
            self.path_to_follow.clear()
        self.target = None
        self.state = Target_Reacher_State.IDLE

    def set_target_and_reset(self, target: Tuple[float, float]):
        self.reset()
        self.target = target

    def on_idle(self):
        if(self.target is not None):
            self.state = Target_Reacher_State.FIND_PATH

    def go_to_dashing(self):
        if(self.target is None):
            self.reset()
            return
        self.motion_controller.set_target(self.target[0], self.target[1])
        self.state = Target_Reacher_State.DASHING

    def on_find_path(self):
        if(self.target is None):
            self.reset()
            return
        current_pos = (self.dave.x, self.dave.y)
        best_path = find_best_path_possible(
            self.topo_map,
            current_pos,
            self.target,
        )
        if(best_path is None):
            self.go_to_dashing()
            return
        self.state = Target_Reacher_State.PATH_FOLLOWING
        self.path_to_follow = transform_node_list_to_point_follow_list(
            best_path)
        self.point_follow.terminate_current_run_and_set_path(
            self.path_to_follow)
        self.point_follow.start_current_path()

    def path_following(self):
        self.point_follow.run(self.dave)
        if(self.point_follow.state == Point_Follow_States.FINISHED):
            self.go_to_dashing()

    def go_to_wall_following(self):
        if(self.target is None):
            self.reset()
            return
        if not(any_wall_detected(self.dave)):
            return
        front_vector = self.dave.get_front_facing_vector().flatten()
        target_vector = np.array(self.target) - \
            np.array((self.dave.x, self.dave.y))  # type: ignore
        cross_product_value = float(np.cross(front_vector, target_vector))
        if(cross_product_value < 0):
            self.state = Target_Reacher_State.RIGHT_WALL_FOLLOWING
        else:
            self.state = Target_Reacher_State.LEFT_WALL_FOLLOWING

    def dashing(self):
        self.motion_controller.go_to_position(
            self.dave, self.point_follow.first_turn_threshold, self.point_follow.linear_threshold)
        if(self.target is None):
            self.reset()
            return
        can_dash = self.dashability_checker.check_dashability(
            self.dave.x, self.dave.y,
            self.target[0], self.target[1]
        )

        if(not can_dash):
            self.go_to_wall_following()

    def left_wall_following(self):
        if(self.target is None):
            self.reset()
            return

        attempt2_left_wall_following(self.dave)
        can_dash = self.dashability_checker.check_dashability(
            self.dave.x, self.dave.y,
            self.target[0], self.target[1]
        )
        if(can_dash):
            self.go_to_dashing()

    def right_wall_following(self):
        if(self.target is None):
            self.reset()
            return

        attempt2_right_wall_following(self.dave)
        can_dash = self.dashability_checker.check_dashability(
            self.dave.x, self.dave.y,
            self.target[0], self.target[1]
        )
        if(can_dash):
            self.go_to_dashing()

    def run(self):
        if(self.state == Target_Reacher_State.FINISHED):
            return
        self.funs_to_run[self.state]()
