from math import radians

NO_WALL = float('inf')


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

    def simple_turn_left(self,vel):
        self.set_velcoity(vel,-vel)

    def simple_turn_right(self,vel):
        self.set_velcoity(-vel,vel)

    def simple_forward(self,vel):
        self.set_velcoity(vel,vel)
    
    def simple_reverse(self,vel):
        self.set_velcoity(-vel,-vel)

class Environment:
    def __init__(self):
        self.collectibles = [(0.0,0.0)]
        self.goals = [(0.0,0.0)]
        
       

def pretty_print_sensor_data(dave: Dave):
    for i, val in enumerate(dave.wall_dis):
        print(f"{i}: {val}\t", end="")
    print()


def update_dave_pose(packet, dave: Dave):
    dave.orientation = radians(packet["robotAngleDegrees"])
    dave.x = packet["robot"][0]
    dave.y = packet["robot"][1]


def update_environment(packet,env:Environment):
    env.collectibles = packet["collectibles"]
    env.goals = packet["goals"]
