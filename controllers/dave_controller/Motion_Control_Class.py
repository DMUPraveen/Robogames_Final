
import numpy as np


class PID:
    def __init__(self, Kp: float, Kd: float, timeste_in_ms: int):
        self.Kp = Kp
        self.Kd = Kd
        self.dt = timeste_in_ms/1000

        # self.accumilated_error = 0 #to be used when implementing the Integral part
        self.previous_error = 0
        self.error = 0
        self.set_point = 0

    def define_set_point(self, set_point: float):
        self.set_point = set_point

    def control(self, measurement: float):
        '''
        returns a control signal in the range [-1,1]
        '''
        self.error = (self.set_point-measurement)
        self.error_derivative = (self.error - self.previous_error) / self.dt
        control_signal = self.Kp * self.error + self.Kd * self.error_derivative

        # dealing with saturation
        if(abs(control_signal) > 1.0):
            control_signal = 1.0 if (control_signal > 0) else -1.0

        self.previous_error = self.error
        return control_signal

    def get_error(self):
        return self.error

    def reset(self):
        self.previous_error = 0
        self.previous_error = 0
        self.set_point = 0


class Motion_Control:
    def __init__(self, timestep):
        self.PD_rotate = PID(0.05, 0.000001, timestep)
        self.PD_linear = PID(0.8, 0.01, timestep)
        self.target_position = np.array([0.0, 0.0])
        self.w_max = 6.28

    def set_target(self, target_position: np.ndarray):
        self.target_position = target_position

    def Rotate(self, dave):
        # initial conditions
        if self.PD_rotate.initial:
            current = self.PD_rotate.angle(dave.orientation)
            self.PD_rotate.Measure(self.rotation_target, current, initial=True)
            self.PD_rotate.initial = False

        ut = self.PD_rotate.PD_controller(
            self.rotation_target, dave.orientation)

        # velocities
        dphi_L = -self.w_max * ut
        dphi_R = self.w_max * ut

        self.dave.set_velocity(dphi_L, dphi_R)

        if round(self.PD_rotate.et0, 2) == 0 and round(self.PD_rotate.dedt, 2) == 0:
            self.PD_rotate.Reset()
            return True
        else:
            return False

    def Linear(self, dave):

        if self.PD_linear.initial:
            X0 = array([dave.x, dave.y])
            Xf = self.lin_target_orientation
            current = 0
            target = norm(Xf[0] - X0[0], Xf[1]-Xf[0])
            self.PD_linear.Measure(target, current)
            self.PD_linear.initial = False

        Xt = dave.orientation
        current = norm(Xt[0] - X0[0], Xt[1] - X0[1])
        ut = ut = self.PD_linear.PD_controller(target, current)

        dphi_L = self.w_max * ut
        dphi_R = self.w_max * ut

        self.dave.set_velocity(dphi_L, dphi_R)

        if round(self.PD_linear.et0, 2) == 0 and round(self.PD_linear.dedt, 2) == 0:
            self.PD_linear.Reset()
            return True
        else:
            return False
