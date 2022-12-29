
import json
from dave_lib import Dave
from Resource_manager import res_man
def get_packets_and_update(receiver,update_on_receive):
    packet = None
    while receiver.getQueueLength() > 0:
        packet = json.loads(receiver.getData().decode('utf-8'))
        receiver.nextPacket()
    if(packet is not None):
        for func in update_on_receive:
            func(packet)


def update_epuck(dave:Dave,res:res_man):
    '''
    Transfers data from dave object to epuck
    '''
    res.left.setVelocity(dave.left_v)
    res.right.setVelocity(dave.right_v)
    