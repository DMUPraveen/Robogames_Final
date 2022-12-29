
import json

def get_packets_and_update(receiver,update_on_receive):
    packet = None
    while receiver.getQueueLength() > 0:
        packet = json.loads(receiver.getData().decode('utf-8'))
        receiver.nextPacket()
    if(packet is not None):
        for func in update_on_receive:
            func(packet)