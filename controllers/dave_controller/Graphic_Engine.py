import pygame
from typing import Dict,Tuple
CELL_SIZE =  10
UNVISITED_CELL_COLOR = (255,255,255)
VISITED_CELL_COLOR = (0,0,255)
BACKGROUND_COLOR = (255,255,255)
WEAK_BOARDER = (122,122,122)
THICK_BOARDER = (0,0,0) 
BOARDER_THICKNESS = 2
WALL_THICKNESS = 4


ROBOT_COLOR = (255,0,0)

NORTH  = 3
SOUTH = 2
EAST = 1
WEST = 0

PIXEL_SPACE_DIRECTION_MAP:Dict[int,Tuple[int,int]] = {
    SOUTH:(0,1),
    NORTH:(0,-1),
    EAST:(1,0),
    WEST:(-1,0),
}
def calculate_origin(screen,grid_width,grid_height=None):
    if(grid_height is None):
        grid_height =grid_width 
    '''
    usefull or calculating a origin in pixel space such that the grid is centerd in the screen
    '''
    grid_width = grid_width*CELL_SIZE
    grid_height = grid_height*CELL_SIZE
    return (
        (screen.get_width() - grid_width)//2,
        (screen.get_height() -  grid_height)//2
    )

def index_to_pixel(origin,i,j):
    x_0 ,y_0 = origin
    return (
        x_0 + j*CELL_SIZE,
        y_0 +i*CELL_SIZE,
    )

def pixel_to_index(origin,x,y):
    i = (x - origin[0])//CELL_SIZE
    j = (y - origin[1])//CELL_SIZE
    return (j,i)

class Graphic_Engine:
    def __init__(self,screen,origin=None):
        self.screen = screen 
        self.origin = origin
    def initialize(self,width,height):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
  
    def draw_cell(self,corner_pos,chara):
        '''
        draws a squre cell of at position i,j with the characatereistc chara
        '''
        visited = bool(chara& (1<<4))

        
        cell_color = VISITED_CELL_COLOR if visited else UNVISITED_CELL_COLOR
    
        #draw the filled cell
        pygame.draw.rect(self.screen,cell_color,(*index_to_pixel(corner_pos,0,0),CELL_SIZE,CELL_SIZE)) 

        #draw weak boarders for clarity
        pygame.draw.rect(self.screen,WEAK_BOARDER,(*index_to_pixel(corner_pos,0,0),CELL_SIZE,CELL_SIZE),BOARDER_THICKNESS) 
    def draw_grid(self,grid):
        itop = lambda x,y : index_to_pixel(self.origin,x,y)
        for i,row in enumerate(grid.grid):
            for j,chara in enumerate(row):
                self.draw_cell(itop(i,j),chara)
        for i,row in enumerate(grid.grid):
            for j,chara in enumerate(row):
               self.draw_walls(itop(i,j),chara)
    def draw_walls(self,corner_pos,chara):
        northwall = bool(chara& (1<<3))
        southwall = bool(chara& (1<<2))
        eastwall = bool(chara& (1<<1))
        westwall = bool(chara& (1<<0))
        walls = [northwall,southwall,eastwall,westwall]
        wall_positions = [
            (index_to_pixel(corner_pos,0,0),index_to_pixel(corner_pos,0,1)),#northwall
            (index_to_pixel(corner_pos,1,0),index_to_pixel(corner_pos,1,1)),#southwall
            (index_to_pixel(corner_pos,0,1),index_to_pixel(corner_pos,1,1)),#eastwall
            (index_to_pixel(corner_pos,0,0),index_to_pixel(corner_pos,1,0))#westwall
        ]
        for wall,(s,e) in zip(walls,wall_positions):
            if(wall):
                pygame.draw.line(self.screen,THICK_BOARDER,s,e,WALL_THICKNESS)
    
    def draw_robot(self,robot):
        position = index_to_pixel(self.origin,robot.i,robot.j)
        position = tuple(i + CELL_SIZE//2 for i in position) #setting the position to the center of the cell rather than the corner
        pygame.draw.circle(self.screen,ROBOT_COLOR,position,CELL_SIZE//2)
        next_pos = tuple(position[i]+PIXEL_SPACE_DIRECTION_MAP[robot.orientation][i]*CELL_SIZE//2 for i in range(2))        
        # print(next_pos)
        pygame.draw.line(self.screen,BACKGROUND_COLOR,position,next_pos)
    
    def run(self,func):

        if(self.screen is None):
            return
        func()        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.screen = None
                return
        pygame.display.flip() 
        self.screen.fill(BACKGROUND_COLOR)
  # Update.
  
  # Draw.
