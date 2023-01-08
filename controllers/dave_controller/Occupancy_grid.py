import numpy as np
from typing import Tuple


class Occupancy_Grid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = np.zeros(
            (height, width)
        )


class Cartesian_to_Grid:
    def __init__(self, scale: float, origin_cell_row: int = 0, origin_cell_column: int = 0) -> None:
        self.scale = scale
        self.o_r: int = origin_cell_row
        self.o_c: int = origin_cell_column

    def catesian_to_grid_position(self, position_x: float, position_y: float) -> Tuple[int, int]:
        '''
        returns the (row,column) corresponding to the positions provided
        '''
        row = int(position_y / self.scale) + self.o_r
        column = int(position_x / self.scale) + self.o_c
        return (row, column)

    def set_cartesian_cordinate_to_grid_origin(self, new_origin_position_x: float, new_origin_position_y: float) -> None:
        pass
