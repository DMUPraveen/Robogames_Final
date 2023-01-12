from numpy import arctan2, deg2rad, array
from math import pi
import numpy as np


def column_vector_to_flat_array(column_vector_pos_2d):
    return tuple(column_vector_pos_2d.flatten())


def flat_array_to_column_vector(flat_position_2d):
    return np.array(flat_position_2d).reshape(2, -1)


def theta_rotated_unit_vector(theta_radians: float):
    '''
        returns [cos(theta),sin(theta)]
    '''
    return np.array([
        np.cos(theta_radians),
        np.sin(theta_radians)
    ]
    ).reshape((2, -1))


def get_rotation_matrix_2d(theta_radians):
    return np.array(
        [
            [np.cos(theta_radians), -np.sin(theta_radians)],
            [np.sin(theta_radians), np.cos(theta_radians)]
        ]
    )


def rotate_2d_vector(vecto_2d: np.ndarray, theta_radians):
    rotation_matrix = get_rotation_matrix_2d(theta_radians)
    return rotation_matrix@vecto_2d.reshape((2, -1))


def x_unit_vector():
    return np.array([[1.0], [0.0]])


def round_angle(theta):
    '''
    converts an angle to the range 0,360 in degrees
    '''
    return (theta % 360.0)


def clamp_radian(theta):
    '''
    clamps an angle given in radians to be between 0 and 2pi

    '''
    return theta % (2*pi)


def convert_angle_radians(angle):
    angle = clamp_radian(angle)
    if(angle < pi):
        return angle

    return angle-2*pi


def convert_angle(theta):
    '''
     Function to convert angles to range in -PI to PI
    '''
    # theta in degrees
    if theta <= 180:
        return deg2rad(theta)

    else:
        return deg2rad(theta-360)


class Rel_Cord:
    def __init__(self):
        self.s_x = 0
        self.s_y = 0
        self.s_o = 0
        self.set = False

    def set_vals(self, x, y, ori):
        if(self.set):
            return
        self.s_x = x
        self.s_y = y
        self.s_o = ori
        self.set = True

    def get_cart(self, x, y):
        prx, pry = (x - self.s_x, y-self.s_y)
        return (-pry, prx)

    def get_ori(self, ori):
        return round_angle(ori - self.s_o)


class Grid_Pos():
    def __init__(self, o_i, o_j, sf):
        self.o_i = o_i
        self.o_j = o_j
        self.scaling_factor = sf

    def grid_pos(self, x, y):
        # print("In grid pos", x, y)
        return (int(x/self.scaling_factor) + self.o_j, -int(y/self.scaling_factor) + self.o_i)

    def grid_pos_float(self, x, y):
        '''
        returns the scaled and transformed x,y to scale and origin of the grid without transfomring the values
        '''

        return ((x/self.scaling_factor) + self.o_j, -(y/self.scaling_factor) + self.o_i)
