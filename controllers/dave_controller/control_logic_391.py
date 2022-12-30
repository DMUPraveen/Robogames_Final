from dave_lib import Dave
from math import atan2
from geometry import convert_angle_radians


def closest_target(targets,current_pos):
    x,y = current_pos
    best_target = 0
    best_dis = float('inf')
    for i,(tx,ty) in enumerate(targets):
        dis = (tx-x)**2+(ty-y)**2
        if(dis < best_dis):
            best_dis = dis
            best_target = i
    return targets[best_target]



def control_logic_basic(dave:Dave,wall_threshold,target):
    front_facing = [0,7]
    target_x,target_y = target
    angle = atan2((target_y-dave.y),(target_x-dave.x))
    difference = convert_angle_radians(angle - dave.orientation)
    print(difference)
    if(any(dave.wall_dis[i] < wall_threshold for i in front_facing)):
        if(difference <0):
            dave.simple_turn_right(1)
        dave.simple_turn_left(1)


    else:
        dave.simple_forward(2)
    if(dave.wall_dis[2]<wall_threshold):
        dave.right_v +=1
    if(dave.wall_dis[5]<wall_threshold):
        dave.left_v +=1

    
  