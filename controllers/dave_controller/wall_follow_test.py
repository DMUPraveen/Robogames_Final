
import pygame
# from Graphic_Engine import Graphic_Engine, calculate_origin
from Resource_manager import res_man
from dave_lib import Dave, update_dave_pose, update_environment, Environment
from communicater import get_packets_and_update, update_epuck
from remote_control import control_dave_via_keyboard
from New_graphic_Engine import Graphic_Engine, draw_dave, draw_grid_view, tracking_grid_view
from Occupancy_grid import Occupancy_Grid, Cartesian_to_Grid, Mapper, get_threshold_based_obstacle_distance_determiner
from wall_following import attempt2_left_wall_following


def main():
    ############################ Resource Management #####################################
    res = res_man()
    dave = Dave()
    env = Environment()
    OCCUPANCY_GRID_WIDTH = 4000
    OCCUPANCY_GRID_HEIGHT = 4000
    OCCUPANCY_GRID_SCALE = 0.004
    occupancy_grid = Occupancy_Grid(
        OCCUPANCY_GRID_WIDTH, OCCUPANCY_GRID_HEIGHT)
    cart_to_grid_pos_converter = Cartesian_to_Grid(
        OCCUPANCY_GRID_SCALE, OCCUPANCY_GRID_WIDTH//2, OCCUPANCY_GRID_HEIGHT//2)
    obstacle_cell_determiner = get_threshold_based_obstacle_distance_determiner(0.2, 0.002
                                                                                )
    mapper = Mapper(occupancy_grid, cart_to_grid_pos_converter,
                    obstacle_cell_determiner, 0.001)
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

    def robot_visualization(screen: pygame.Surface): return draw_dave(
        screen, dave, ROBOT_RADIUS, SCREEN_WIDTH-ROBOT_OFFSET, SCREEN_HEIGHT-ROBOT_OFFSET)

    def render_grid_subset(screen: pygame.Surface):
        # print(occupancy_grid.grid[1900:2100, 1450:1550])
        draw_grid_view(
            screen, occupancy_grid.grid[1900:2100, 1450:1550], 4, (0, 0))

    def render_grid_tracking_subset(screen: pygame.Surface):
        tracking_position = cart_to_grid_pos_converter(dave.x, dave.y)
        grid_view = tracking_grid_view(tracking_position, occupancy_grid, 100)
        draw_grid_view(screen, grid_view, 4, (0, 0))

    all_visualizations = [
        Graphic_Engine.clear_screen,
        robot_visualization,
        render_grid_tracking_subset,

    ]
    ######################################################################################

    ############################## Main Loop #############################################
    while res.robot.step(res.timestep) != -1:
        print(dave)
        print(dave.left_v, dave.right_v)
        packet_recieved = get_packets_and_update(
            res.receiver, update_on_receive)
        update_epuck(dave, res)
        mapper.update_map(dave)
        vis.run(all_visualizations)
        attempt2_left_wall_following(dave)
    # print(dave)
######################################################################################
