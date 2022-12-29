import pygame
from Graphic_Engine import Graphic_Engine,calculate_origin
from grid import Grid 
from Resource_manager import res_man
from dave_lib import Dave,update_dave_pose
from communicater import get_packets_and_update,update_epuck

res = res_man()
vis = Graphic_Engine(None,(0,0))
gg = [[0]*10 for _ in range(10)]
grid = Grid(gg)

dave  = Dave()

vis.initialize(400,400)
vis.origin = calculate_origin(vis.screen,10,10)

update_dave_on_packet = lambda packet: update_dave_pose(packet,dave)
update_on_receive = [update_dave_on_packet]
def visulizer():
    vis.draw_grid(grid)

dave.set_velcoity(0.1,-0.1)
while res.robot.step(res.timestep) != -1:
    get_packets_and_update(res.receiver,update_on_receive)
    update_epuck(dave,res) 
    vis.run(visulizer)