from controller import Robot
from controller import Motor


class res_man:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(10)
        
        self.left:Motor = self.robot.getDevice('left wheel motor')
        self.right:Motor = self.robot.getDevice('right wheel motor')

        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)
