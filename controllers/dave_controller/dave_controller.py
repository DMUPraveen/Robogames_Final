import pygame
from Graphic_Engine import Graphic_Engine,calculate_origin
from grid import Grid 
from Resource_manager import res_man
from dave_lib import Dave,update_dave_pose,update_environment,Environment
from communicater import get_packets_and_update,update_epuck
from control_logic_391 import control_logic_basic,closest_target
from mapping import determine_wall_states,pprint_wall_state,Map
from geometry import Rel_Cord,Grid_Pos

res = res_man()
vis = Graphic_Engine(None,(0,0))
grid_size = 40
gg = [[0]*grid_size for _ in range(grid_size)]
grid = Grid(gg)

dave  = Dave()
env = Environment()

vis.initialize(400,400)
vis.origin = calculate_origin(vis.screen,grid_size,grid_size)
relcod = Rel_Cord()
gridpos = Grid_Pos(grid_size//2,grid_size//2,0.02)

update_dave_on_packet = lambda packet: update_dave_pose(packet,dave)
update_env_on_packet = lambda packet: update_environment(packet,env)
update_on_receive = [update_dave_on_packet,update_env_on_packet]
mm = Map(grid,relcod,gridpos)
def visulizer():
    vis.draw_grid(grid)

dave.set_velcoity(0.1,-0.1)
while res.robot.step(res.timestep) != -1:
    packet_recieved = get_packets_and_update(res.receiver,update_on_receive)
    update_epuck(dave,res) 
    vis.run(visulizer)
    dave.simple_turn_left(0.1)
    if(not relcod.set):
        if(not packet_recieved):
            continue
        print("setting relcod values")
        relcod.set_vals(dave.x,dave.y,dave.orientation)
    try:
        wall_state = determine_wall_states(dave,0.05)
        mm.update_grid(dave)
        pprint_wall_state(wall_state)
        dave.simple_forward(1)
        control_logic_basic(dave,0.05,closest_target(env.collectibles,(dave.x,dave.y)))
    except Exception:
        print("An error occured")
        print(relcod.s_o,relcod.s_x,relcod.s_y)
        print(dave.x,dave.y)
    # print(dave.orientation)
    
