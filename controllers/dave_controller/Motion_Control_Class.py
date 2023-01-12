from numpy.linalg import norm
from numpy import arctan2 , deg2rad 


# ---------------------- testing ------------------------------------
from controller import Robot
from controller import Receiver
import json
from numpy import array 

class PID:
    def __init__( self , Kp , Kd , timestep):
        self.Kp = Kp
        self.Kd = Kd
        self.timestep = timestep
        self.dt = timestep/1000
        self.et_total = 0

        # error values
        self.et = None
        self.et0 = None
        self.dedt = None

    def angle( self , theta):
         # theta in degrees
        if theta <= 180:
            return deg2rad(theta)        
        else:
            return deg2rad(theta-360)
    
    def Measure( self , target , current , initial = False):

        self.et = target - current      
          
        if initial:
            self.et_total = self.et0 = self.et
        else:
            self.et_total += self.et

    def PD_controller( self , target , current ):

        self.Measure( target , current)

        self.dedt = ( self.et - self.et0 ) / self.dt
        ut = self.Kp * self.et + self.Kd * self.dedt
        
        ut *= 2        

        # speedup slower part
        if abs(self.et) < abs(self.et_total/1.5):
            ut *= 4 

        # saturation
        if abs(ut ) > 1:
            ut = ut / abs(ut)

        self.et0 = self.et
        return ut 

# Function for rotation
def Rotate(robot , L_motor , R_motor, receiver ,X0,Xf,orientation,timestep):
    w_max = 6.28
    Kp = 0.05
    Kd = 0.000001
    PD_rot = PID( Kp , Kd , timestep)

    # initial conditions
    current = PD_rot.angle(orientation)
    target = float(arctan2([ Xf[1]-X0[1] ],[ Xf[0]-X0[0] ]))
    PD_rot.Measure( target , current , initial = True)

    while robot.step(timestep) != -1:
        # data updating from the reciver
      if receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))
        #print(data['robot'],data['robotAngleDegrees'])
        current = PD_rot.angle(data['robotAngleDegrees'])
        receiver.nextPacket() 

      ut = PD_rot.PD_controller( target , current)  
        
      dphi_L = -w_max * ut
      dphi_R = w_max * ut
            
      L_motor.setVelocity(dphi_L)
      R_motor.setVelocity(dphi_R)              
        
      print(current,target,PD_rot.et,PD_rot.dedt , ut)      
        
      if round(PD_rot.dedt,2) == 0 and round(PD_rot.et0,2) == 0:
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)            
            break  
        
      PD_rot.et0 = PD_rot.et
    return True


def Linear(robot , L_motor , R_motor, receiver ,X0,Xf):
  # parameters
   Kp = 0.8
   Kd = 0.01
   w_max = 6.28

   PD_lin = PID( Kp , Kd , timestep)

   # initial conditions
   target = float(norm([ Xf[0]-X0[0] , Xf[1]-X0[1]] ))
   current = 0
   PD_lin.Measure( current , target ,initial = True)

   while robot.step(timestep) != -1:
        # data updating from the reciver
      if receiver.getQueueLength() > 0:
        data = json.loads(receiver.getData().decode('utf-8'))
        #print(data['robot'],data['robotAngleDegrees'])
        Xt = data['robot']
        current = norm([Xt[0]-X0[0] , Xt[1]-X0[1]])
        receiver.nextPacket() 

      ut = PD_lin.PD_controller( target , current)  
        
      dphi_L = w_max * ut
      dphi_R = w_max * ut
            
      L_motor.setVelocity(dphi_L)
      R_motor.setVelocity(dphi_R)              
        
      print(X0,Xt,Xf,PD_lin.et,PD_lin.dedt , ut)      
        
      if round(PD_lin.dedt,2) == 0 and round(PD_lin.et0,2) == 0:
            L_motor.setVelocity(0)
            R_motor.setVelocity(0)            
            break  
        
      PD_lin.et0 = PD_lin.et
   return True



# ---------- Testing ----------------------
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
xf = 0.5
yf = -0.5
Xf = array([xf,yf])

# initial conditions
x0 = -2.0876
y0 = -0.0154828
X0 = array([x0,y0])
orientation = 0
# --------------------- Rotation and Linera Motion -------------------------------------------  
Rotate(robot , L_motor , R_motor, receiver ,X0,Xf,orientation,timestep)
Linear(robot , L_motor , R_motor, receiver ,X0,Xf)