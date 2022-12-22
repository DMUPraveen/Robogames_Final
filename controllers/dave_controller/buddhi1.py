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
prox_sensors=[]
def printvalues(robot):
    for i in range(8):
        sensor_name="ps"+str(i)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[i].enable(timestep)
    while robot.step(timestep) != -1:
        for i in range(8):
            print("i: {},val: {}".format(i,prox_sensors[i].getValue()),end=" ")
        print()
printvalues(robot)       
while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        print(json.loads(receiver.getData().decode('utf-8')))
        receiver.nextPacket()
    pass
