import pygame
from Graphic_Engine import Graphic_Engine,calculate_origin
from grid import Grid 
from Resource_manager import res_man
from dave_lib import Dave,update_dave_pose,update_environment,Environment
from communicater import get_packets_and_update,update_epuck
from remote_control import control_dave_via_keyboard

res = res_man()

dave  = Dave()
env = Environment()


update_dave_on_packet = lambda packet: update_dave_pose(packet,dave)
update_env_on_packet = lambda packet: update_environment(packet,env)
update_on_receive = [update_dave_on_packet,update_env_on_packet]


while res.robot.step(res.timestep) != -1:
    packet_recieved = get_packets_and_update(res.receiver,update_on_receive)
    update_epuck(dave,res) 
    control_dave_via_keyboard(res.keyboard,dave)
    print(dave)
     
