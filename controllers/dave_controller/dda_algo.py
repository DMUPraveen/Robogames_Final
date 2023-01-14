from typing import Tuple, Callable
import numpy as np


def perform_dda(direction_vector: Tuple[float, float],
                start_scaled_to_grid_units: Tuple[float, float],
                stopping_condition: Callable[[Tuple[int, int], float], bool],
                on_new_cell: Callable[[Tuple[int, int]], None],
                ) -> Tuple[Tuple[int, int], float]:
    '''
    performs the dda algorithm on a grid

    direction_vector: vector giving the direction of the ray -- does not need to be a unit vector 
    start_scaled_to_grid_units: (x,y) values sclaed such that one grid cell is 1 unit
    stopping_condition: condition to be stopped Takes in (x,y), distance
    on_new_cell: what to do on a new cell (will be performed to the stopping cell as well)

    returns: the cell which the algorithm stopped and the distance traveled thus far
    '''
    np_dir_vec = np.array(direction_vector)
    dir_unit_vec = np_dir_vec/np.linalg.norm(np_dir_vec)
    # distance traveled along the direction for one unit in x
    Sx: float = abs(1/dir_unit_vec[0])
    # distnace traveled along the direction for one unit in y
    Sy: float = abs(1/dir_unit_vec[1])
    # The tile that is currently inhabited
    current_tile = list(map(int, start_scaled_to_grid_units))

    x_ray_length: float = 0.0
    y_ray_length: float = 0.0
    step = [0, 0]
    if(dir_unit_vec[0] < 0):
        step[0] = -1
        x_ray_length = (start_scaled_to_grid_units[0]-current_tile[0])*Sx
    else:
        step[0] = 1
        x_ray_length = ((current_tile[0]+1)-start_scaled_to_grid_units[0])*Sx

    if(dir_unit_vec[1] < 0):
        step[1] = -1
        y_ray_length = (start_scaled_to_grid_units[1]-current_tile[1])*Sy
    else:
        step[1] = 1
        y_ray_length = ((current_tile[1]+1)-start_scaled_to_grid_units[1])*Sy

    stop_flag = False
    distance: float = 0.0
    tuple_current_tile = tuple(current_tile)
    while(not stop_flag):
        tuple_current_tile = tuple(current_tile)
        if(x_ray_length < y_ray_length):
            current_tile[0] += step[0]
            distance = x_ray_length
            x_ray_length += Sx
        else:
            current_tile[1] += step[1]
            distance = x_ray_length
            y_ray_length += Sy
        on_new_cell(tuple_current_tile)
        if(stopping_condition(tuple_current_tile, distance)):
            stop_flag = True
    return tuple_current_tile, distance
