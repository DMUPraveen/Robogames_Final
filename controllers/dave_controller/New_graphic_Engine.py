
import pygame
from typing import Iterable, Callable, Any, Optional, Tuple
from dave_lib import Dave
import numpy as np
from geometry import column_vector_to_flat_array, flat_array_to_column_vector
from Occupancy_grid import Occupancy_Grid


class Graphic_Engine:

    Type_drawing_function = Callable[[Optional[pygame.Surface]], None]

    def __init__(self):
        self.screen: Optional[pygame.Surface] = None

    def initialize(self, width, height):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))

    @staticmethod
    def clear_screen(screen: pygame.surface, clear_color=(255, 255, 255)):
        screen.fill(clear_color)

    def run(self, list_of_draw_calls: Iterable[Type_drawing_function]):

        if(self.screen is None):
            return
        for draw_call in list_of_draw_calls:
            draw_call(self.screen)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.screen = None
                return
        pygame.display.flip()


DAVE_BODY_COLOR = (0, 0, 255)
SENSOR_COLOR = (255, 0, 0)
ACTIVATED_SENSOR_COLOR = (0, 255, 0)
ACTIVATION_COLOR_THRESHOL = 0.1
FRONT_LINE_COLOR = (255, 255, 255)

# Means the length of the sensor line draw is 2*radius of robot that is extends a radius more...
SENSOR_SCALE = 2


def column_vector_to_pygame_position(column_vector_pos_2d: np.ndarray):
    return column_vector_to_flat_array(column_vector_pos_2d)


def pygame_position_to_2d_column_vecotor(position):
    return flat_array_to_column_vector(position)


def draw_dave(screen: pygame.Surface, robot: Dave, radius: int, position_x: int, position_y: int):
    position = (position_x, position_y)
    pygame.draw.circle(screen, DAVE_BODY_COLOR, position, radius)

    for sensor_unit_vector, dis in zip(robot.get_sensor_unit_vectors(), robot.wall_dis):

        sensor_pos = pygame_position_to_2d_column_vecotor(
            position) + sensor_unit_vector*radius*SENSOR_SCALE
        intger_sensor_pos = column_vector_to_pygame_position(sensor_pos)
        sensor_col = ACTIVATED_SENSOR_COLOR if(
            dis < ACTIVATION_COLOR_THRESHOL) else SENSOR_COLOR
        pygame.draw.line(screen, sensor_col, position, intger_sensor_pos)
    # print(robot.get_front_facing_vector())
    front_pos = column_vector_to_pygame_position(
        robot.get_front_facing_vector()*radius+pygame_position_to_2d_column_vecotor(position))
    pygame.draw.line(screen, FRONT_LINE_COLOR, position, front_pos)


VISITED_COLOR = (0, 0, 255)
BLOCKED_COLOR = (255, 0, 0)
UNVISITED_COLOR = (255, 255, 255)
VALUE_TO_COLOR_MAP = {
    0: UNVISITED_COLOR,
    1: BLOCKED_COLOR,
    2: VISITED_COLOR
}
CELL_OUTLINE_COLOR = (0, 0, 0)


def draw_grid_view(screen: pygame.Surface, grid_view: np.ndarray, cell_size: int, start: Tuple[int, int]):
    def get_corner(i, j): return (cell_size*i+start[0], cell_size*j+start[1])
    def get_color(val): return VALUE_TO_COLOR_MAP[val]
    for row_index, row in enumerate(grid_view):
        for column_index, value in enumerate(row):

            pygame.draw.rect(
                screen,
                get_color(value),
                (get_corner(row_index, column_index), (cell_size, cell_size)),
            )

            pygame.draw.rect(
                screen,
                CELL_OUTLINE_COLOR,
                (get_corner(row_index, column_index), (cell_size, cell_size)),
                width=1
            )


def tracking_grid_view(grid_pos: Tuple[int, int], occupancy_grid: Occupancy_Grid, window_size: int) -> np.ndarray:
    '''
    Returns a grid view such that the grid_pos provided is always in the middle in a window of size window_size
    '''
    row = grid_pos[0]
    column = grid_pos[1]

    min_row = max((row - window_size//2), 0)
    max_row = min((row+window_size//2), occupancy_grid.height-1)
    min_column = max((column-window_size//2), 0)
    max_column = min((column+window_size//2), occupancy_grid.width-1)

    return occupancy_grid.grid[min_row:max_row, min_column:max_column]
