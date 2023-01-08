import pygame
from Graphic_Engine import Graphic_Engine, calculate_origin
from Resource_manager import res_man
from dave_lib import Dave, update_dave_pose, update_environment, Environment
from communicater import get_packets_and_update, update_epuck
from remote_control import control_dave_via_keyboard
from New_graphic_Engine import Graphic_Engine, draw_dave

############################ Resource Management #####################################
res = res_man()
dave = Dave()
env = Environment()
#######################################################################################


############################ Setting up the Resources and Communication################
def update_dave_on_packet(packet): return update_dave_pose(packet, dave)


def update_env_on_packet(packet): return update_environment(packet, env)


update_on_receive = [update_dave_on_packet, update_env_on_packet]
#######################################################################################


########################### Debug Visualization Setup ##################################
vis = Graphic_Engine()
SCREEN_WIDTH = 400
SCREEN_HEIGHT = 400
vis.initialize(SCREEN_WIDTH, SCREEN_HEIGHT)

ROBOT_RADIUS = 20
ROBOT_OFFSET = ROBOT_RADIUS*2


def robot_visualization(screen): return draw_dave(
    screen, dave, ROBOT_RADIUS, SCREEN_WIDTH-ROBOT_OFFSET, SCREEN_HEIGHT-ROBOT_OFFSET)


all_visualizations = [
    Graphic_Engine.clear_screen,
    robot_visualization,
]
######################################################################################


############################## Main Loop #############################################
while res.robot.step(res.timestep) != -1:
    packet_recieved = get_packets_and_update(res.receiver, update_on_receive)
    update_epuck(dave, res)
    control_dave_via_keyboard(res.keyboard, dave)
    vis.run(all_visualizations)
    print(dave)
######################################################################################
