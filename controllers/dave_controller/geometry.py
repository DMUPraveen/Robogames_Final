from dave_lib import Dave
from numpy import arctan2, deg2rad, array
from math import pi

def round_angle(theta):
    '''
    converts an angle to the range 0,360 in degrees
    '''
    return (theta%360.0)

def clamp_radian(theta):
    '''
    clamps an angle given in radians to be between 0 and 2pi
    
    '''
    return theta% (2*pi)
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
        prx,pry =  (x - self.s_x, y-self.s_y)
        return (-pry,prx)

    def get_ori(self, ori):
        return round_angle(ori - self.s_o)


class Grid_Pos():
    def __init__(self, o_i, o_j, sf):
        self.o_i = o_i
        self.o_j = o_j
        self.scaling_factor = sf

    def grid_pos(self, x, y):
        print("In grid pos", x,y)
        return (int(x/self.scaling_factor) + self.o_j, -int(y/self.scaling_factor) + self.o_i)


