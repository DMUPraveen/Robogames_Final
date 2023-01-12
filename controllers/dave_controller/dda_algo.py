from typing import Tuple, Callable
import numpy as np


def perform_dda(direction_vector: Tuple[float, float],
                start_scaled_to_grid_units: Tuple[float, float],
                stopping_condition: Callable[[int, int], bool],
                on_new_cell: Callable[[int, int], None],
                maxdistnace: float) -> None:
    np_dir_vec = np.array(direction_vector)
    dir_unit_vec = np_dir_vec/np.linalg.norm(np_dir_vec)
    # distance traveled along the direction for one unit in x
    Sx: float = abs(1/dir_unit_vec[0])
    # distnace traveled along the direction for one unit in y
    Sy: float = abs(1/dir_unit_vec[1])
    # The tile that is currently inhabited
    current_tile = tuple(map(int, start_scaled_to_grid_units))

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
    while(not stop_flag):
        pass
