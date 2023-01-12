from numpy.linalg import norm
from numpy import array , pi


class PID:
    def __init__( self , Kp , Kd , timestep):
        self.Kp = Kp
        self.Kd = Kd
        self.timestep = timestep
        self.dt = timestep/1000
        self.et_total = 0
        self.initial = True

        # error values
        self.et = None
        self.et0 = None
        self.dedt = None

    def angle( self , theta):
        
        if theta <= pi:
            return theta       
        else:
            return theta-2*pi
    
    def Measure( self , target , current , initial = False):

        self.et = target - current      
          
        if initial:
            self.et0 = self.et
        

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


    def Reset(self):
        self.et_total = 0
        self.dedt = 0
        self.et = 0
        self.et0 = 0
        self.initial = True



class Motion_Control:
    def __init__( self, timestep):
        self.PD_rotate = PID( 0.05 , 0.000001 , timestep )
        self.PD_linear = PID( 0.8 , 0.01 , timestep )
        self.linear_target_orientation = None
        self.rotation_target = None          
        self.w_max = 6.28

    def set_target(self,target_rot,target_lin):
        self.lin_target_orientation = target_lin
        self.rotation_target = target_rot   
    

    def Rotate( self ,dave ):
        # initial conditions
        if self.PD_rotate.initial:            
            current = self.PD_rotate.angle(dave.orientation)
            self.PD_rotate.Measure( self.rotation_target , current , initial = True )
            self.PD_rotate.initial = False

        ut = self.PD_rotate.PD_controller( self.rotation_target , dave.orientation )

        # velocities
        dphi_L = -self.w_max * ut
        dphi_R = self.w_max * ut
            
        self.dave.set_velocity( dphi_L , dphi_R )

        if round(self.PD_rotate.et0, 2) == 0 and round(self.PD_rotate.dedt , 2 ) == 0:
            self.PD_rotate.Reset()
            return True 
        else:
            return False
    
    def Linear( self,dave):

        if self.PD_linear.initial:
            X0 = array([dave.x , dave.y])
            Xf = self.lin_target_orientation
            current = 0
            target = norm( Xf[0] - X0[0] , Xf[1]-Xf[0])
            self.PD_linear.Measure( target , current)
            self.PD_linear.initial = False

        Xt = dave.orientation
        current = norm( Xt[0] - X0[0] , Xt[1] - X0[1])
        ut = ut = self.PD_linear.PD_controller( target , current )

        dphi_L = self.w_max * ut
        dphi_R = self.w_max * ut

        self.dave.set_velocity( dphi_L , dphi_R )

        if round(self.PD_linear.et0, 2) == 0 and round(self.PD_linear.dedt , 2 ) == 0:            
            self.PD_linear.Reset()
            return True 
        else:
            return False