from controller import Robot
from controller import Receiver
import json

robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(10)

left = robot.getDevice('left wheel motor')
right = robot.getDevice('right wheel motor')

left.setPosition(float("inf"))
right.setPosition(float("inf"))

left.setVelocity(0.0)
right.setVelocity(0.0)






class Dave:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.left_v = 0
        self.right_v = 0
    

    def set_velcoity(self,left_v,righ_v):
        self.left_v = left_v
        self.right_v = righ_v
    


def update_dave_pose(packet,dave:Dave):
    dave.orientation = packet["robotAngleDegrees"]
    dave.x = packet["robot"][0]
    dave.y = packet["robot"][1]
    
while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        print(json.loads(receiver.getData().decode('utf-8')))
        receiver.nextPacket()
    pass
    