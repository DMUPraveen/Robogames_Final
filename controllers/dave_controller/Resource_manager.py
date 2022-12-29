from controller import Robot


class res_man:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(10)
        
        self.left = self.robot.getDevice('left wheel motor')
        self.right = self.robot.getDevice('right wheel motor')

        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)
