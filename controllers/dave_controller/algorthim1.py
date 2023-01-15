from typing import Iterable, List, Tuple, Deque, Set, Dict, Optional
from dave_lib import Dave, update_dave_pose, update_environment, Environment
from Occupancy_grid import Occupancy_Grid, Cartesian_to_Grid, Mapper, get_true_distance_with_maximum_free_distance
from Motion_Control_Class import Motion_Control
from path_planning import Point_Follow, Point_Follow_States, Topological_Map,\
    find_best_path_possible, transform_node_list_to_point_follow_list, Dashability_Checker,\
    find_nearest_unvisited_cell, find_connectible_potins
from wall_following import attempt2_left_wall_following, attempt2_right_wall_following, any_wall_detected, left_front_wall_detected, right_front_wall_detected
from enum import Enum
import numpy as np
from random import random


class Target_Reacher_State(Enum):
    IDLE = 0
    FIND_PATH = 1
    PATH_FOLLOWING = 2
    DASHING = 3
    LEFT_WALL_FOLLOWING = 4
    RIGHT_WALL_FOLLOWING = 5
    FINISHED = 6
    STUCK = 7
    ESCAPE_PATH = 8
    SUPER_STUCK = 9


class Timer():
    def __init__(self):
        self.time: int = 0

    def reset(self):
        self.time = 0

    def tick(self):
        self.time += 1

    def get_time(self):
        return self.time


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
        self.start_time = 0
        self.current_topo_cell = None

        self.funs_to_run = {
            Target_Reacher_State.IDLE: self.on_idle,
            Target_Reacher_State.DASHING: self.dashing,
            Target_Reacher_State.FIND_PATH: self.on_find_path,
            Target_Reacher_State.PATH_FOLLOWING: self.path_following,
            Target_Reacher_State.LEFT_WALL_FOLLOWING: self.left_wall_following,
            Target_Reacher_State.RIGHT_WALL_FOLLOWING: self.right_wall_following,
            Target_Reacher_State.ESCAPE_PATH: self.on_escape_path,
            Target_Reacher_State.STUCK: self.on_stuck,
        }
        self.timer = Timer()

    def reset(self):
        if(self.path_to_follow is not None):
            self.path_to_follow.clear()
        self.target = None
        self.state = Target_Reacher_State.IDLE
        self.start_time = self.timer.get_time()
        self.timer.tick()

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

    LOOP_THRESHOLD = 10
    STUCK_THRESHOLD = 300

    def run(self):
        if(self.state == Target_Reacher_State.SUPER_STUCK):
            return
        new_topo_cell = self.topo_map.cartesian_to_grid(
            self.dave.x, self.dave.y)
        # if(self.current_topo_cell is not None):
        #     row, column = self.current_topo_cell
        #     last_time = self.topo_map.visited_times[row][column]
        #     print(row, column, last_time, self.timer.get_time())
        self.timer.tick()
        self.funs_to_run[self.state]()
        if(self.state == Target_Reacher_State.PATH_FOLLOWING):
            return
        if(new_topo_cell != self.current_topo_cell or self.current_topo_cell is None):
            self.current_topo_cell = new_topo_cell
            row, column = self.current_topo_cell
            last_time = self.topo_map.visited_times[row][column]
            new_time = self.timer.get_time()

            if (last_time > self.start_time):
                if (new_time - last_time) > self.LOOP_THRESHOLD:
                    # print("loop->stuck")
                    # self.i_am_super_stuck()
                    # return
                    # self.state = Target_Reacher_State.STUCK
                    pass
            self.topo_map.visited_times[row][column] = new_time
        else:
            # print("still in the same cell")
            row, column = self.current_topo_cell
            last_time = self.topo_map.visited_times[row][column]
            if(last_time > self.start_time):

                if(self.timer.get_time()-last_time) > self.STUCK_THRESHOLD:
                    print(last_time, self.timer.get_time())
                    # print("stuck")
                    # if(self.state == Target_Reacher_State.ESCAPE_PATH):
                    #     self.state = Target_Reacher_State.SUPER_STUCK
                    #     return
                    self.i_am_super_stuck()
                    return

                    self.state = Target_Reacher_State.STUCK

        if(self.state == Target_Reacher_State.FINISHED):
            return

    def is_super_stuck(self):
        return (self.state == Target_Reacher_State.SUPER_STUCK)

    def i_am_super_stuck(self):
        self.state = Target_Reacher_State.SUPER_STUCK

    def on_stuck(self):
        if(self.current_topo_cell is None):
            # raise Exception("current cell is does not exist")
            self.i_am_super_stuck()
            return

        ret = find_nearest_unvisited_cell(
            self.topo_map.visited_times, self.current_topo_cell, self.start_time)
        if(ret is None):
            # raise Exception("No unvisited cells")
            self.i_am_super_stuck()
            return
        unvisited, parent = ret
        connectible_points = find_connectible_potins(
            parent, unvisited, self.topo_map, self.dashability_checker)
        if(connectible_points is None):
            # raise Exception("No")
            self.i_am_super_stuck()
            return
        path = find_best_path_possible(
            self.topo_map, (self.dave.x, self.dave.y), (-1.929, -0.173))
        if(path is None):
            self.i_am_super_stuck()
            # raise Exception("No path found")
            return

            # print(self.topo_map.topo_grid[parent[0]][parent[1]])
        self.state = Target_Reacher_State.ESCAPE_PATH
        escape_path = transform_node_list_to_point_follow_list(path)
        escape_path.append(connectible_points[1])
        escape_path.append(connectible_points[2])
        self.point_follow.terminate_current_run_and_set_path(escape_path)
        self.point_follow.start_current_path()
        print(unvisited, parent, self.topo_map.cartesian_to_grid(
            self.dave.x, self.dave.y))

    def on_escape_path(self):
        if(self.point_follow.state == Point_Follow_States.FINISHED):
            target = self.target
            if(target is None):
                self.reset()
            else:
                self.set_target_and_reset(target)
            return
        print("on escapep path")

        # if(any_wall_detected(self.dave)):
        #     # print("walls detected")
        #     # self.on_stuck()
        #     return

        #     self.state = Target_Reacher_State.DASHING
        #     return
        # self.on_stuck()
        self.point_follow.run(self.dave)


GOAL = 0
COLLECTIBLE = 1


MAX_RUPEES_LIMIT = 3000


class Supermachine:
    def __init__(self, target_reacher: Target_Reacher, dave: Dave, env: Environment):
        self.target_reacher = target_reacher
        self.dave = dave
        self.timer = Timer()
        self.env = env
        self.current_target = None
        self.ss = None
        self.target_type = COLLECTIBLE
        self.previous_rupees = 0
        self.previous_dollars = 0

    def get_closest(self, targets: List[Tuple[float, float]]):
        return min(targets,
                   key=lambda target: (
                       target[0]-self.dave.x)**2 + (target[1]-self.dave.y)**2
                   )

    def check_and_set_targets(self):
        state_change = False
        if(self.current_target is None):
            state_change = True
        elif(self.previous_dollars != self.env.dollars):
            state_change = True
        elif(self.previous_rupees != self.env.rupees):
            state_change = True

        if(not state_change):
            return

        if(self.env.rupees < MAX_RUPEES_LIMIT):
            self.target_type = COLLECTIBLE
        else:
            self.target_type = GOAL

        if(self.target_type == GOAL):
            self.current_target = self.get_closest(self.env.goals)
        else:
            self.current_target = self.get_closest(self.env.collectibles)

        self.target_reacher.set_target_and_reset(self.current_target)
        self.previous_dollars = self.env.dollars
        self.previous_rupees = self.env.rupees

    def run_target_reacher(self, stuck_evasion_reverse_time: int):
        if(self.current_target is None):
            return
        if(self.ss != self.target_reacher.state):
            self.ss = self.target_reacher.state
            print(self.ss)

        if(self.target_reacher.is_super_stuck()):
            v = -1.0
            omega = random()*1.0
            self.dave.set_velcoity(v+omega, v-omega)
            self.timer.tick()
            if(self.timer.get_time() > stuck_evasion_reverse_time):
                self.timer.reset()
                if(self.target_reacher.target is None):
                    self.target_reacher.reset()
                else:
                    self.target_reacher.set_target_and_reset(
                        self.target_reacher.target)
        else:
            self.target_reacher.run()
