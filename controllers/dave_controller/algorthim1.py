from typing import Iterable, List, Tuple, Deque, Set, Dict, Optional
from dave_lib import Dave, update_dave_pose, update_environment, Environment
from Occupancy_grid import Occupancy_Grid, Cartesian_to_Grid, Mapper, get_true_distance_with_maximum_free_distance
from Motion_Control_Class import Motion_Control
from path_planning import Point_Follow, Point_Follow_States, Topological_Map,\
    find_best_path_possible, transform_node_list_to_point_follow_list, Dashability_Checker,\
    find_nearest_unvisited_cell, find_connectible_potins, find_nearest_visited_cell, bfs_find
from wall_following import attempt2_left_wall_following, attempt2_right_wall_following, any_wall_detected, left_front_wall_detected, right_front_wall_detected
from enum import Enum
import numpy as np
from random import random


def calculate_delta(current_pos, previous_pos):
    delta = (current_pos[0]-previous_pos[0])**2 + \
        (current_pos[1]-previous_pos[1])**2
    return delta


def random_point(xm, ym, sm):
    return(
        xm+random()*sm,
        ym+random()*sm
    )


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
        self.tried = set()

    def reset(self):
        if(self.path_to_follow is not None):
            self.path_to_follow.clear()
        self.target = None
        self.state = Target_Reacher_State.IDLE
        # self.start_time = self.timer.get_time()
        self.timer.tick()

    def set_target_and_reset(self, target: Tuple[float, float]):
        self.reset()
        self.target = target

    def on_idle(self):
        print("we are on idle")
        if(self.target is not None):
            self.state = Target_Reacher_State.FIND_PATH

    def go_to_dashing(self):
        if(self.target is None):
            self.reset()
            return
        self.motion_controller.set_target(self.target[0], self.target[1])
        self.state = Target_Reacher_State.DASHING

    def on_find_path(self):
        print("we are in find path")
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
        print("following")
        self.point_follow.run(self.dave)
        if(self.point_follow.state == Point_Follow_States.FINISHED):
            print("finished")
            self.go_to_dashing()

    def go_to_wall_following(self):
        if(self.target is None):
            self.reset()
            return
        # if not(any_wall_detected(self.dave)):
        #     return
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
        if(self.target is None):
            return

        # if(
        #     self.dashability_checker.check_dashability(
        #         self.dave.x, self.dave.y,
        #         self.target[0], self.target[1]
        #     ) and (self.state == Target_Reacher_State.ESCAPE_PATH or self.state == Target_Reacher_State.PATH_FOLLOWING)
        # ):
        #     self.go_to_dashing()
        #     return
        # print(self.state)
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
        if(self.state == Target_Reacher_State.PATH_FOLLOWING or self.state == Target_Reacher_State.ESCAPE_PATH or self.state == Target_Reacher_State.DASHING):
            return
        if(new_topo_cell != self.current_topo_cell or self.current_topo_cell is None):
            self.current_topo_cell = new_topo_cell
            row, column = self.current_topo_cell
            last_time = self.topo_map.visited_times[row][column]
            new_time = self.timer.get_time()

            if (last_time > self.start_time):
                if (new_time - last_time) > self.LOOP_THRESHOLD:
                    print("loop->stuck")
                    # self.i_am_super_stuck()
                    # return
                    self.state = Target_Reacher_State.STUCK
                    pass
            self.topo_map.visited_times[row][column] = new_time
        else:
            # print("still in the same cell")
            row, column = self.current_topo_cell
            last_time = self.topo_map.visited_times[row][column]
            if(last_time > self.start_time):

                if(self.timer.get_time()-last_time) > self.STUCK_THRESHOLD:
                    # print(last_time, self.timer.get_time())
                    # print("stuck")
                    # if(self.state == Target_Reacher_State.ESCAPE_PATH):
                    #     self.state = Target_Reacher_State.SUPER_STUCK
                    #     return
                    # self.i_am_super_stuck()
                    # return

                    # self.state = Target_Reacher_State.STUCK
                    pass

        if(self.state == Target_Reacher_State.FINISHED):
            return

    def is_super_stuck(self):
        return (self.state == Target_Reacher_State.SUPER_STUCK)

    def i_am_super_stuck(self):
        # print("super stuck")
        self.state = Target_Reacher_State.SUPER_STUCK

    def on_stuck(self):
        print("running on stuck")
        if(self.target is None):
            raise Exception("Target is None")
        cell = find_nearest_visited_cell(
            self.topo_map, self.topo_map.cartesian_to_grid(*self.target), self.tried)
        if(cell is None):
            print("Internal super stuck")
            self.i_am_super_stuck()
            return
        vc, uc = cell
        xm, ym, sm = self.topo_map.cartesian_to_grid.get_random_points_center_and_range(
            *uc)
        new_target = random_point(xm, ym, sm)
        new_path = find_best_path_possible(
            self.topo_map, (self.dave.x, self.dave.y), new_target)
        point = (self.dave.x, self.dave.y)
        if(new_path is None):

            if(self.dashability_checker.check_dashability(*point, *new_target)):
                self.tried.add(self.topo_map.cartesian_to_grid(*self.target))
                print("cannot go")
                return
        print(f"Resetting target from {self.target} to {new_target}")
        self.tried.add(self.topo_map.cartesian_to_grid(*self.target))
        self.set_target_and_reset(new_target)

    def on_escape_path(self):
        if(self.point_follow.state == Point_Follow_States.FINISHED):
            self.go_to_dashing()
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
        self.stuck_timer = Timer()
        self.previous_pos = (dave.x, dave.y)

    def get_closest(self, targets: List[Tuple[float, float]]):
        return min(targets,
                   key=lambda target: (
                       target[0]-self.dave.x)**2 + (target[1]-self.dave.y)**2
                   )

    def change_target(self):
        if(self.target_type == GOAL):
            print("setting new goal")
            self.current_target = self.get_closest(self.env.goals)
        else:
            # print("setting new collectible")
            self.current_target = self.get_closest(self.env.collectibles)

        self.target_reacher.set_target_and_reset(self.current_target)

    def check_and_set_targets(self):
        state_change = False
        # print("money", self.previous_dollars, self.previous_rupees,
        #       self.env.rupees, self.env.dollars)
        if(self.current_target is None):
            state_change = True
        elif(self.previous_dollars != self.env.dollars):
            state_change = True
        elif abs(self.previous_rupees - self.env.rupees) > 100 and self.target_type != GOAL:
            state_change = True

        self.previous_dollars = self.env.dollars
        self.previous_rupees = self.env.rupees
        if(not state_change):
            return
        print("state has been changed")

        if(self.env.rupees == 0):
            self.target_type = COLLECTIBLE
        else:
            self.target_type = GOAL
        self.change_target()

    STUCK_DELTA = 0.0005
    STUCK_TIME = 200
    TARGET_REACHED_THRESHOLD = 0.001

    def run_target_reacher(self, stuck_evasion_reverse_time: int):
        self.stuck_timer.tick()
        # print(self.stuck_timer.get_time())
        if(self.stuck_timer.get_time() > self.STUCK_TIME):
            # print("super timer tick detected")
            current_pos = self.dave.x, self.dave.y
            print(current_pos, self.previous_pos)
            self.stuck_timer.reset()
            delta = calculate_delta(current_pos, self.previous_pos)
            print(delta)
            if(delta < self.STUCK_DELTA and (self.target_reacher.state != Target_Reacher_State.PATH_FOLLOWING or self.target_reacher.start_time != Target_Reacher_State.ESCAPE_PATH)):
                print("super stuck detected from timer")
                # self.target_reacher.tried.clear()
                self.target_reacher.i_am_super_stuck()
            self.previous_pos = current_pos

        if(self.current_target is None):
            return
        if(self.ss != self.target_reacher.state):
            self.ss = self.target_reacher.state
            # print(self.ss)

        if(self.target_reacher.is_super_stuck()):
            # print("running reovery")
            v = -1.0
            omega = random()*1.0
            self.dave.set_velcoity(v+omega, v-omega)
            self.timer.tick()
            if(self.timer.get_time() > stuck_evasion_reverse_time):
                print("finished evasion")
                self.timer.reset()
                # self.stuck_timer.time = -self.STUCK_TIME
                if(self.target_reacher.target is None):
                    self.target_reacher.reset()
                else:
                    self.target_reacher.set_target_and_reset(
                        self.target_reacher.target)
                self.dave.simple_forward(1.0)
            return
        delta = calculate_delta(
            self.target_reacher.target, (self.dave.x, self.dave.y))
        if(delta < self.TARGET_REACHED_THRESHOLD):
            # print("target_reached here")
            self.change_target()
        else:
            self.target_reacher.run()
