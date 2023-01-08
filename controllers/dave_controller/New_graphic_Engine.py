
import pygame
from typing import Iterable,Callable,Any,Optional,Tuple
from dave_lib import Dave
import numpy as np
class Graphic_Engine:

    Type_drawing_function = Callable[[Optional[pygame.Surface]],None]
    def __init__(self):
        self.screen:Optional[pygame.Surface] = None 
    def initialize(self,width,height):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
  
    
    def run(self,list_of_draw_calls: Iterable[Type_drawing_function]):

        if(self.screen is None):
            return
        for draw_call in list_of_draw_calls:
            draw_call(self.screen)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.screen = None
                return
        pygame.display.flip() 
        

DAVE_BODY_COLOR = (0,0,255)
SENSOR_COLOR = (255,0,0)
def draw_dave(screen:pygame.Surface,robot:Dave,radius:int, position_x:int, position_y:int):
    pygame.draw.circle(screen,DAVE_BODY_COLOR,[position_x,position_y],radius)
    unit_vector = np.array([0,1])
    for sensor_angle in robot.get_sensors_orientations():



