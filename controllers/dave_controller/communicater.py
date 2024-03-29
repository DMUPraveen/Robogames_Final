
import json
from dave_lib import Dave, NO_WALL
from Resource_manager import res_man
from distance_function import transform_sensor_reading


def get_packets_and_update(receiver, update_on_receive):
    packet = None

    while receiver.getQueueLength() > 0:
        data = receiver.getData()
        if(type(data) != str):
            data = data.decode('utf-8')
        packet = json.loads(data)
        # print(packet)
        receiver.nextPacket()
    if(packet is not None):
        for func in update_on_receive:
            func(packet)
        return True
    return False


SCALING_FACTOR = 10
THRESHOLD = 75


def transform_reading_old(reading):
    if(reading < THRESHOLD):
        return NO_WALL
    return SCALING_FACTOR/(reading-THRESHOLD)


def positionofwalls(prox_sensors):
    return list(map(transform_sensor_reading, (sensor.getValue() for sensor in prox_sensors)))


MAX_VELOCITY = 6


def update_epuck(dave: Dave, res: res_man):
    '''
    Transfers data from dave object to epuck
    '''

    if(abs(dave.left_v) > MAX_VELOCITY):
        dave.left_v = dave.left_v/abs(dave.left_v)*MAX_VELOCITY
    if(abs(dave.right_v) > MAX_VELOCITY):
        dave.right_v = dave.left_v/abs(dave.right_v)*MAX_VELOCITY
    res.left.setVelocity(dave.left_v)
    res.right.setVelocity(dave.right_v)
    dave.wall_dis = positionofwalls(res.prox_sensors)
