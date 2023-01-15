from typing import Tuple, Callable, Iterable, Optional
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
    stopping_condition: condition to be stopped Takes in (x:int,y:int), distance
    on_new_cell: what to do on a new cell  takes in  (x:int,y:int)

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
    # print(distance)
    while(not stop_flag):
        on_new_cell(tuple_current_tile)
        if(x_ray_length < y_ray_length):
            current_tile[0] += step[0]
            distance = x_ray_length
            x_ray_length += Sx
        else:
            current_tile[1] += step[1]
            distance = y_ray_length
            y_ray_length += Sy
        tuple_current_tile = tuple(current_tile)
        if(stopping_condition(tuple_current_tile, distance)):
            stop_flag = True
    return tuple_current_tile, distance


def testing_code():
    from New_graphic_Engine import Graphic_Engine, draw_grid_view
    import pygame
    import numpy as np
    vis = Graphic_Engine()
    vis.initialize(400, 400)
    grid = np.zeros((50, 50), dtype=int)

    def draw_visuals(screen: Optional[pygame.Surface]):
        if(screen is None):
            return
        draw_grid_view(screen, grid, 7, (0, 0))

    visual_list: Iterable[Graphic_Engine.Type_drawing_function] = [
        draw_visuals]

    def on_cell_visit(cell_indices: Tuple[int, int]):
        x, y = cell_indices
        row, column = y, x
        grid[row, column] = 2

    def stopping_condition(cell_indices: Tuple[int, int], distance: float):
        x, y = cell_indices
        row, column = y, x
        number_of_rows = grid.shape[0]
        number_of_columns = grid.shape[1]
        if not(0 <= row < number_of_rows and 0 <= column < number_of_columns):
            return True
        if(grid[row, column] == 1):
            return True
        return False

    start_position = (20, 20)
    direction_vector = (10.5-20, 8.5-20)
    grid[8, 10] = 1
    print(perform_dda(direction_vector, start_position,
          stopping_condition, on_cell_visit))
    while(vis.screen is not None):
        vis.run(visual_list)


if __name__ == "__main__":
    testing_code()
