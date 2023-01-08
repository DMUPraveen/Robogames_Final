from math import radians
from math import pi as PI
from geometry import clamp_radian, rotate_2d_vector, get_rotation_matrix_2d, theta_rotated_unit_vector
import numpy as np

NO_WALL = float('inf')

_RAW_SENSOR_POSITIONS = [
    1.27,
    0.77,
    0.00,
    5.21,
    4.21,
    3.1459,
    2.37,
    1.87,
    4.71239,
]

SENSOR_ORIENTATIONS = [
    (val - PI/2) % (2*PI) for val in _RAW_SENSOR_POSITIONS
]

ZERO_ANGLE_UNIT_VECTOR = np.array([1, 0])


def sensor_orientations(dave_orientation: float):
    return tuple(clamp_radian(angle+dave_orientation) for angle in SENSOR_ORIENTATIONS)


SENSOR_UNIT_VECTORS = [rotate_2d_vector(
    np.array([1, 0]), rotated_angle) for rotated_angle in SENSOR_ORIENTATIONS]


class Dave:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.left_v = 0.0
        self.right_v = 0.0
        self.wall_dis = [NO_WALL]*8

    def set_velcoity(self, left_v, righ_v):
        self.left_v = left_v
        self.right_v = righ_v

    def simple_turn_left(self, vel):
        self.set_velcoity(-vel, vel)

    def simple_turn_right(self, vel):
        self.set_velcoity(vel, -vel)

    def simple_forward(self, vel):
        self.set_velcoity(vel, vel)

    def simple_stop(self):
        self.set_velcoity(0.0, 0.0)

    def simple_reverse(self, vel):
        self.set_velcoity(-vel, -vel)

    def __repr__(self) -> str:
        return self.__str__()

    def __str__(self) -> str:
        return f"x: {self.x:.3f}\t  y: {self.y:.3f}\t  orientation: {self.orientation:.3f}"

    def get_sensor_unit_vectors(self):
        rotation_matrix = get_rotation_matrix_2d(self.orientation)
        return [rotation_matrix@sensor_unit_vector for sensor_unit_vector in SENSOR_UNIT_VECTORS]

    def get_front_facing_vector(self):
        return theta_rotated_unit_vector(self.orientation)


class Environment:
    def __init__(self):
        self.collectibles = [(0.0, 0.0)]
        self.goals = [(0.0, 0.0)]


def pretty_print_sensor_data(dave: Dave):
    for i, val in enumerate(dave.wall_dis):
        print(f"{i}: {val}\t", end="")
    print()


def update_dave_pose(packet, dave: Dave):
    dave.orientation = radians(packet["robotAngleDegrees"])
    dave.x = packet["robot"][0]
    dave.y = packet["robot"][1]


def update_environment(packet, env: Environment):
    env.collectibles = packet["collectibles"]
    env.goals = packet["goals"]
