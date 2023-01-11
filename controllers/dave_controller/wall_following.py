from dave_lib import SENSOR_UNIT_VECTORS, Dave
import numpy as np

DISTANCE_TO_PS5 = 0.031
DISTANCE_TO_PS6 = 0.033
AVERAGE_RADIUS = np.mean([DISTANCE_TO_PS5, DISTANCE_TO_PS6])


def simple_left_wall_following(dave):
    left_wall_distance = dave.wall_dis[5]
    left_up_wall_distance = dave.wall_dis[6]
    forward_wall0_distance = dave.wall_dis[0]
    forward_wall7_distance = dave.wall_dis[7]

    if forward_wall0_distance <= 0.04 or forward_wall7_distance <= 0.04:
        dave.simple_turn_right(6.28)
    else:
        if left_wall_distance <= 0.06:
            if left_up_wall_distance >= 0.01:
                dave.simple_forward(6.28)
            else:
                dave.simple_turn_right(6.28)
        elif left_wall_distance > 0.06 and left_up_wall_distance > 0.06:
            dave.simple_turn_left(6.28)
        else:
            dave.simple_forward(6.28)


def calculate_angle_left_wall_following(dave):
    wall_distance_from_sensor5 = dave.wall_dis[5]+DISTANCE_TO_PS5
    wall_distance_from_sensor6 = dave.wall_dis[6]+DISTANCE_TO_PS6
    v5 = wall_distance_from_sensor5*SENSOR_UNIT_VECTORS[5]
    v6 = wall_distance_from_sensor6*SENSOR_UNIT_VECTORS[6]
    difference = v6-v5
    angle = -np.arctan2(difference[1], difference[0])
    return(angle)


def calculate_perpendicular_distance_left_wall_following(dave):
    wall_distance_from_sensor5 = dave.wall_dis[5]+DISTANCE_TO_PS5
    wall_distance_from_sensor6 = dave.wall_dis[6]+DISTANCE_TO_PS6
    v5 = wall_distance_from_sensor5*SENSOR_UNIT_VECTORS[5]
    v6 = wall_distance_from_sensor6*SENSOR_UNIT_VECTORS[6]
    dot_product_v5_v6 = np.dot(v5.flatten(), v6.flatten())
    lamba = (np.linalg.norm(v5)**2-dot_product_v5_v6) / \
        (np.linalg.norm(v6)**2-dot_product_v5_v6)
    print(lamba)
    perpendicular_distance_vector = (v5+lamba*v6)/(lamba+1)
    perpendicular_distance = np.linalg.norm(perpendicular_distance_vector)
    return(perpendicular_distance-AVERAGE_RADIUS)


def attempt2_left_wall_following(dave: Dave):
    if dave.wall_dis[5] <= 100 and dave.wall_dis[6] <= 100:
        alpha = 1
        beta = -1
        gamma = 1
        n_angle = calculate_angle_left_wall_following(dave)/np.pi*2
        n_distance = calculate_perpendicular_distance_left_wall_following(
            dave)/0.07
        error = alpha*n_angle + beta*n_distance
        omega = gamma*error
        v = 2
        lv = v+omega
        rv = v-omega
        dave.set_velcoity(lv, rv)
        print(f"{n_angle=} {n_distance=} {error=} {omega=}")

    else:
        dave.simple_turn_left(6.28)
