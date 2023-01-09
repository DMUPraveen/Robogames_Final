
from dave_lib import Dave,sensor_orientations
from math import pi
from geometry import clamp_radian, convert_angle_radians, Rel_Cord, Grid_Pos
from grid import NORTH, SOUTH, EAST, WEST, Grid


ANGLE_RANGE = (-pi/4, pi/4)


def transform_range(ran, offset):
    return sorted(tuple(clamp_radian(i+offset) for i in ran))


angle_ranges = {
    WEST: (pi/4, pi/2+pi/4),
    EAST: (pi+pi/4, 2*pi-pi/4),
    NORTH:  (2*pi-pi/4, pi/4),
    SOUTH:  (pi/2+pi/4, pi+pi/4),
}

dir_names = {
    NORTH: "NORTH",
    SOUTH: "SOUTH",
    EAST: "EAST",
    WEST: "WEST"
}


def within_range(ran, angle):

    alpha = ran[0]
    beta = ran[1]
    da = abs(convert_angle_radians(angle - alpha))
    db = abs(convert_angle_radians(beta - angle))
    # print(da,db)
    return (da < pi/2 and db < pi/2)


def pprint_wall_state(wall_state):
    for i in range(4):
        print(f"{dir_names[i]}: {wall_state[i]}", end="\t\t")
    print()


def determine_wall_states(dave: Dave, threshold):
    oris = sensor_orientations(dave.orientation)
    wall_state = [False]*4
    for i, (angle, dis) in enumerate(zip(oris, dave.wall_dis)):
        # print(f"{i}: {angle}: {dis}")
        if(dis > threshold):
            continue
        for dire, anran in angle_ranges.items():
            # print(angle,anran)
            if(within_range(anran, angle)):
                wall_state[dire] = True
    return wall_state


class Map:
    def __init__(self, grid: Grid, rel_cod: Rel_Cord, grid_pos: Grid_Pos):
        self.rel_cod: Rel_Cord = rel_cod
        self.grid_pos: Grid_Pos = grid_pos
        self.grid: Grid = grid
        self.wall_threshold = 0.05

    def get_current_cell(self, x, y):
        return self.grid_pos.grid_pos(*self.rel_cod.get_cart(x, y))

    def update_grid(self, dave: Dave):
        walls = determine_wall_states(dave, self.wall_threshold)

        x,y = self.get_current_cell(dave.x,dave.y)
        self.grid.set_visited(y,x)
        for dire, wall in enumerate(walls):
            if(wall):
                self.grid.set_wall_directly(
                    y,x,
                    dire)
