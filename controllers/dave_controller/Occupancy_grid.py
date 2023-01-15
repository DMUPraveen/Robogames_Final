import numpy as np
from typing import Tuple, Callable
from geometry import column_vector_to_flat_array, flat_array_to_column_vector
from dave_lib import Dave
from dda_algo import perform_dda


class Cartesian_to_Grid:
    def __init__(self, scale: float, origin_cell_row: int = 0, origin_cell_column: int = 0) -> None:
        self.scale = scale
        self.o_r: int = origin_cell_row  # basically the y=0 position
        self.o_c: int = origin_cell_column  # basically the x=0 position

    def __call__(self, position_x, position_y):
        return self.catesian_to_grid_position(position_x, position_y)

    def catesian_to_grid_position(self, position_x: float, position_y: float) -> Tuple[int, int]:
        '''
        returns the (row,column) corresponding to the positions provided
        '''
        row = int(position_y / self.scale + self.o_r)
        column = int(position_x / self.scale + self.o_c)
        return (row, column)

    def scale_xy_to_grid_scale(self, position_x: float, position_y: float) -> Tuple[float, float]:
        '''
        scales and translates provided x and y values to the grid origin please note the following:
            1. This function does not convert to integer domain
            2. This function only scales and transforms relatice to the scale and origin of the grid itself
            3. Does not return row,column ... but scaled and transformed x,y (They are not swapped)
        '''
        return (
            position_x/self.scale+self.o_c,
            position_y/self.scale+self.o_r
        )

    def scale_lengths(self, length_in_meters: float):
        '''
        converts lengths to grid_scale lengths 
        '''
        return length_in_meters/self.scale


class Occupancy_Grid:
    OBSTACLE = 1
    CELL_VISITED = 2

    def __init__(self, width: int, height: int,):
        self.width = width
        self.height = height
        self.grid = np.zeros(
            (height, width),
            int
        )

    def set_obstacle(self, row, column):
        # print(f"obstacle @{row,column}")
        self.grid[row, column] = self.OBSTACLE

    def set_visited(self, row, column):
        if(self.grid[row, column] != 0):
            return
        # print(f"visited @{row,column}")
        self.grid[row, column] = self.CELL_VISITED


DistanceSensorToWallDistance = Callable[[float], Tuple[bool, float]]


class Mapper:
    def __init__(self, occupancy_grid: Occupancy_Grid, cart_to_grid_pos: Cartesian_to_Grid, obstacle_cell_determiner: DistanceSensorToWallDistance, raymarching_delta):
        self.occupancy_grid = occupancy_grid
        self.cart_to_grid_pos = cart_to_grid_pos
        self.obstacle_cell_determiner = obstacle_cell_determiner
        # used to ray cast and find the next cell if the cell that the wall is detected also happens to be is also the current cell
        self.raymarching_delta = raymarching_delta

    def update_map(self, dave: Dave):
        current_grid_position = self.cart_to_grid_pos(dave.x, dave.y)
        current_colum_vector_2d = flat_array_to_column_vector((dave.x, dave.y))
        for distance, sensor_unit_vector in zip(dave.get_distances(), dave.get_sensor_unit_vectors()):
            is_wall, offset = self.obstacle_cell_determiner(distance)
            obstacle_grid_position = None
            while (obstacle_grid_position is None) or (obstacle_grid_position == current_grid_position):
                obstacle_position_column_vector_2d = offset * \
                    sensor_unit_vector + current_colum_vector_2d
                obstacle_grid_position = self.cart_to_grid_pos(
                    *column_vector_to_flat_array(obstacle_position_column_vector_2d))
                offset += self.raymarching_delta
            row, column = obstacle_grid_position
            if(is_wall):
                self.occupancy_grid.set_obstacle(row, column)

            else:
                self.occupancy_grid.set_visited(row, column)

        self.occupancy_grid.set_visited(*current_grid_position)

    def get_stopping_condition_for_mapping_with_dda(self, offset: float, current_grid_position: Tuple[int, int]):
        def stopping_condition_for_mapping_with_dda(grid_cell: Tuple[int, int], distance: float):
            row, column = grid_cell[1], grid_cell[0]
            if not(0 <= row < self.occupancy_grid.height and 0 <= column < self.occupancy_grid.width):
                return True
            if(row == current_grid_position[0] and column == current_grid_position[1]):
                return False
            if(distance > offset):
                return True
            return False
        return stopping_condition_for_mapping_with_dda

    def on_new_cell_for_mapping_with_dda(self, grid_cell: Tuple[int, int]) -> None:
        row, column = grid_cell[1], grid_cell[0]
        if not(0 <= row < self.occupancy_grid.height and 0 <= column < self.occupancy_grid.width):
            return
        self.occupancy_grid.set_visited(row, column)

    def mapping_with_dda(self, dave: Dave):
        current_position_grid = self.cart_to_grid_pos(
            dave.x, dave.y)  # a grid position (row,column)
        current_scaled_position_cartesian = self.cart_to_grid_pos.scale_xy_to_grid_scale(
            dave.x, dave.y)
        for distance, sensor_unit_vector in zip(dave.get_distances(), dave.get_sensor_unit_vectors()):
            is_wall, offset = self.obstacle_cell_determiner(distance)
            grid_offset = self.cart_to_grid_pos.scale_lengths(offset)
            stopping_condition_for_mapping_with_dda = self.get_stopping_condition_for_mapping_with_dda(
                grid_offset, current_position_grid)
            final_cell, _ = perform_dda(
                column_vector_to_flat_array(sensor_unit_vector),
                current_scaled_position_cartesian,
                stopping_condition_for_mapping_with_dda,
                self.on_new_cell_for_mapping_with_dda,
            )
            if(is_wall):
                row, column = final_cell[1], final_cell[0]
                self.occupancy_grid.set_obstacle(row, column)


def get_threshold_based_obstacle_distance_determiner(threshold: float, constant_offset: float) -> DistanceSensorToWallDistance:
    def threshold_based_distance_determiner(distance: float):
        if(distance > threshold):
            return (False, 0)
        return (True, constant_offset)

    return threshold_based_distance_determiner


def get_true_distance_obstacle_determiner(threshold: float) -> DistanceSensorToWallDistance:
    def get_true_distance_determiner(distance: float):
        if(distance > threshold):
            return (False, 0)
        return (True, distance)
    return get_true_distance_determiner


def get_true_distance_with_maximum_free_distance(threshold: float, max_free_distance_threshold) -> DistanceSensorToWallDistance:
    assert(threshold > max_free_distance_threshold)

    def true_distance_with_maximum_free_distance(distance: float):
        if(distance > threshold):
            return (False, max_free_distance_threshold)
        return (True, distance)
    return true_distance_with_maximum_free_distance
