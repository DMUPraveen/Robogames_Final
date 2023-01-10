import numpy as np
from typing import Tuple, Callable
from geometry import column_vector_to_flat_array, flat_array_to_column_vector
from dave_lib import Dave


class Cartesian_to_Grid:
    def __init__(self, scale: float, origin_cell_row: int = 0, origin_cell_column: int = 0) -> None:
        self.scale = scale
        self.o_r: int = origin_cell_row
        self.o_c: int = origin_cell_column

    def __call__(self, position_x, position_y):
        return self.catesian_to_grid_position(position_x, position_y)

    def catesian_to_grid_position(self, position_x: float, position_y: float) -> Tuple[int, int]:
        '''
        returns the (row,column) corresponding to the positions provided
        '''
        row = int(position_y / self.scale) + self.o_r
        column = int(position_x / self.scale) + self.o_c
        return (row, column)


class Occupancy_Grid:
    def __init__(self, width: int, height: int,):
        self.width = width
        self.height = height
        self.grid = np.zeros(
            (height, width),
            int
        )

    def set_obstacle(self, row, column):
        # print(f"obstacle @{row,column}")
        self.grid[row, column] = 1

    def set_visited(self, row, column):
        # print(f"visited @{row,column}")
        self.grid[row, column] = 2


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


def get_threshold_based_obstacle_distance_determiner(threshold: float, constant_offset: float) -> DistanceSensorToWallDistance:
    def threshold_based_distance_determiner(distance: float):
        if(distance > threshold):
            return (False, 0)
        return (True, constant_offset)

    return threshold_based_distance_determiner


