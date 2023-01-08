
import pygame
from typing import Iterable, Callable, Any, Optional, Tuple
from dave_lib import Dave
import numpy as np
from geometry import column_vector_to_flat_array, flat_array_to_column_vector


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
    print(robot.get_front_facing_vector())
    front_pos = column_vector_to_pygame_position(
        robot.get_front_facing_vector()*radius+pygame_position_to_2d_column_vecotor(position))
    pygame.draw.line(screen, FRONT_LINE_COLOR, position, front_pos)
