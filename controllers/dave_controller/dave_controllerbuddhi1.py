from controller import Robot
from controller import Receiver
import json

def runrobot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.00
    receiver = robot.getDevice("receiver")
    receiver.enable(10)
    
    left = robot.getDevice('left wheel motor')
    right = robot.getDevice('right wheel motor')
    
    left.setPosition(float("inf"))
    right.setPosition(float("inf"))
    
    left.setVelocity(max_speed)
    right.setVelocity(max_speed)
    
    prox_sensors=[]
    for i in range(8):
        sensor_name="ps"+str(i)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[i].enable(timestep)
    
    while robot.step(timestep) != -1:
        for i in range(8):
            print("i: {},val: {}".format(i,prox_sensors[i].getValue()),end=" ")
            print()
        left_wall = prox_sensors[5].getValue() > 90
        front_wall = prox_sensors[7].getValue() > 90
        
        left_speed = max_speed
        right_speed = max_speed
        
        if front_wall:
            print("Turn right in place")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("drive forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turn  left")
                left_speed = max_speed/8
                right_speed = max_speed
                
        left.setVelocity(left_speed)
        right.setVelocity(right_speed) 
        
        #while receiver.getQueueLength() > 0:
            #print(json.loads(receiver.getData().decode('utf-8')))
           # receiver.nextPacket()
       # pass
       
if __name__ == "__main__":
    robot = Robot()
    runrobot(robot)
