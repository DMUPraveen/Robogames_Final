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

while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        print(json.loads(receiver.getData().decode('utf-8')))
        receiver.nextPacket()
    pass
