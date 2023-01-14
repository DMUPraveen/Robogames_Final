
import pygame
# from Graphic_Engine import Graphic_Engine, calculate_origin
from Resource_manager import res_man
from dave_lib import Dave, update_dave_pose, update_environment, Environment
from communicater import get_packets_and_update, update_epuck
from remote_control import control_dave_via_keyboard, actions
from New_graphic_Engine import Graphic_Engine, draw_dave, draw_grid_view, tracking_grid_view
from Occupancy_grid import Occupancy_Grid, Cartesian_to_Grid, Mapper, get_true_distance_with_maximum_free_distance
from Motion_Control_Class import Motion_Control
from path_planning import Point_Follow, Point_Follow_States


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
    obstacle_cell_determiner = get_true_distance_with_maximum_free_distance(
        0.06, 0.02)
    mapper = Mapper(occupancy_grid, cart_to_grid_pos_converter,
                    obstacle_cell_determiner, 0.001)
    motion_controller = Motion_Control(res.timestep)
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

    ############################Code for Path Planning####################################
    point_follower = Point_Follow(motion_controller, 0.1, 0.003)

    test_path = [
        (-1.929, -0.173),
        (-1.904, -0.282),
        (-1.946, -0.406),
        (-1.691, -0.559),
        (-1.566, -0.946),
        (-1.551, -1.127),
        (-1.551, -1.383),
        (-1.431, -1.498),
    ]
    point_follower.terminate_current_run_and_set_path(test_path)
    point_follower.start_current_path()
    ######################################################################################

    ############################## Main Loop #############################################

    while res.robot.step(res.timestep) != -1:
        packet_recieved = get_packets_and_update(
            res.receiver, update_on_receive)
        update_epuck(dave, res)
        mapper.mapping_with_dda(dave)
        vis.run(all_visualizations)
        print(point_follower.state)
        point_follower.run(dave)
        print(dave)
        if(point_follower.state == Point_Follow_States.FINISHED):
            test_path = list(reversed(test_path))
            point_follower.terminate_current_run_and_set_path(test_path)
            point_follower.start_current_path()
        # print(dave)
        # control_dave_via_keyboard(res.keyboard, dave)
        # print(dave.get_distances()[0])
        # print(dave)
    ######################################################################################
