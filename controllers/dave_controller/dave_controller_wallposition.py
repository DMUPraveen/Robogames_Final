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

def positionofwalls(robot):
    prox_sensors=[]
    for i in range(8):
        sensor_name="ps"+str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i].enable(timestep)
    #for i in range(8):
            #print("i: {},val: {}".format(i,prox_sensors[i].getValue()),end=" ")
            #print()
    list1=[prox_sensors[i].getValue()>70 for i in range(8)]
    dict1={}
    #print(list1)
    for i in range(8):
        if list1[i]==True:
            dict1[i]=10/(prox_sensors[i].getValue()-70)
        else:
            dict1[i]=-1
    print(dict1)

while robot.step(timestep) != -1:
    positionofwalls(robot)
    while receiver.getQueueLength() > 0:
        print(json.loads(receiver.getData()))
        receiver.nextPacket()
    pass
    