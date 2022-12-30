from controller import Robot
from controller import Receiver
import json
from numpy.linalg import norm
from numpy import arctan2 , deg2rad , array 

# parameters
w_max = 6.28

# Function to convert angles to range in -PI to PI
def angle( theta ):
    # theta in degrees
    if theta <= 180:
        return deg2rad(theta)
        
    else:
        return deg2rad(theta-360)
       

# Function for rotation
def Rotate(robot , L_motor , R_motor, receiver ,X0,Xf,orientation):
   # parameters
   Kp = 0.05
   Kd = 0.000001
    
   
   global w_max
   global timestep
   dt = timestep / 1000

   # initial conditions
   current = angle(orientation)
   target = float(arctan2([ Xf[1]-X0[1] ],[ Xf[0]-X0[0] ]))
   et_total = et_0 = target - current
   
   while robot.step(timestep) != -1:     
      # data updating from the reciver
      if receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))
        #print(data['robot'],data['robotAngleDegrees'])
        current = angle(data['robotAngleDegrees'])
        receiver.nextPacket() 

      
      et = target - current
     
      dedt = (et-et_0)/dt
      ut = Kp * et + Kd * dedt 
      ut *= 2    

      if abs(ut ) > 1:
        ut = ut / abs(ut)

      if abs(et) < abs(et_total/1.5):
        ut *= 4     

      dphi_L = -w_max * ut
      dphi_R = w_max * ut
           
      L_motor.setVelocity(dphi_L)
      R_motor.setVelocity(dphi_R)              
      
      print(current,target,et,dedt , ut)      
      
      if round(dedt,2) == 0 and round(et_0,2) == 0:
         L_motor.setVelocity(0)
         R_motor.setVelocity(0)
         break  
      et_0 = et
      

# Function for Linear Motion
def Linear(robot , L_motor , R_motor, receiver ,X0,Xf):
   # parameters
   Kp = 0.8
   Kd = 0.01
      
   global w_max
   global timestep
   dt = timestep / 1000

   # initial conditions
   target = float(norm([ Xf[0]-X0[0] , Xf[1]-X0[1]] ))
   current = 0
   et_total = et_0 = target - current
   
  
   while robot.step(timestep) != -1:

    # data updating from the reciver
    if receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))
        #print(data['robot'],data['robotAngleDegrees'])
        Xt = data['robot']
        receiver.nextPacket() 

    current = norm([Xt[0]-X0[0] , Xt[1]-X0[1]])

    et = target - current

    dedt = (et-et_0)/dt
    ut = Kp * et + Kd * dedt

    ut *= 2        

    if abs(et) < abs(et_total/1.5):
        ut *= 4 

    if abs(ut ) > 1:
        ut = ut / abs(ut)

    dphi_L = w_max * ut
    dphi_R = w_max * ut     
    
    L_motor.setVelocity(dphi_L)
    R_motor.setVelocity(dphi_R)

    print(X0,Xt,Xf,et)

    if round(dedt,1) == 0 and round(et_0,1) == 0:            
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)
            break
    et_0 = et

# initialisation
robot = Robot()
timestep = int(robot.getBasicTimeStep())
receiver = robot.getDevice("receiver")
receiver.enable(10)

L_motor = robot.getDevice('left wheel motor')
R_motor = robot.getDevice('right wheel motor')

L_motor.setPosition(float("inf"))
R_motor.setPosition(float("inf"))

L_motor.setVelocity(0.0)
R_motor.setVelocity(0.0)

# ------------------ inputs ---------------------------------------

# target
xf = 1
yf = 0.5
Xf = array([xf,yf])

# initial conditions
x0 = -2.0876
y0 = -0.0154828
X0 = array([x0,y0])
orientation = 0
# --------------------- Rotation and Linera Motion -------------------------------------------  
    
Rotate(robot , L_motor , R_motor, receiver ,X0,Xf,orientation)
Linear(robot , L_motor , R_motor, receiver ,X0,Xf)