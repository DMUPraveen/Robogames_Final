from dave_lib import Dave


def round_angle(a):
    return a %360.0

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
        return (x - self.s_x, y-self.s_y)
    
    def get_ori(self,ori):
        return round_angle(ori - self.s_o)

class Grid_Pos():
    def __init__(self,o_i,o_j,sf):
        self.o_i = o_i
        self.o_j = o_j 
        self.scaling_factor = sf

    def grid_pos(self,x,y):
        return (x//self.scaling_factor -self.o_j,y//self.scaling_factor -self.o_i)

    

    

def sensor_to_wall_data(dave: Dave):
    ori = dave.orientation
