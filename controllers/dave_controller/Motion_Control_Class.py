
import numpy as np
from dave_lib import Dave


class PID:
    def __init__(self, Kp: float, Kd: float, timestep_in_ms: int):
        self.Kp = Kp
        self.Kd = Kd
        self.dt = timestep_in_ms/1000

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
    def __init__(self, timestep_in_ms: int):
        self.pd_rotate = PID(1, 0, timestep_in_ms)
        self.pd_linear = PID(12, 0.01, timestep_in_ms)
        self.target_position = np.array([0.0, 0.0])
        self.w_max = 6.00
        self.rotate_max = 4.0

    def set_target(self, x: float, y: float):
        self.target_position = np.array([x, y])
        self.pd_linear.define_set_point(0)
        self.pd_rotate.define_set_point(0)

    # def Rotate(self, dave):
    #     # initial conditions
    #     if self.PD_rotate.initial:
    #         current = self.PD_rotate.angle(dave.orientation)
    #         self.PD_rotate.Measure(self.rotation_target, current, initial=True)
    #         self.PD_rotate.initial = False

    #     ut = self.PD_rotate.PD_controller(
    #         self.rotation_target, dave.orientation)

    #     # velocities
    #     dphi_L = -self.w_max * ut
    #     dphi_R = self.w_max * ut

    #     self.dave.set_velocity(dphi_L, dphi_R)

    #     if round(self.PD_rotate.et0, 2) == 0 and round(self.PD_rotate.dedt, 2) == 0:
    #         self.PD_rotate.Reset()
    #         return True
    #     else:
    #         return False

    def go_to_position(self, dave: Dave, first_turn_threshold, linear_threshold: float):
        '''
        will go to position with rotating first most of the way first and then starting the linear motion

        first_turn_threshold: threshold of the error in angle to start the linear motion (Until this is reached linear motion will not start)
        linear_threshold: threshold in the error in the linear distance that is tolerated
        '''

        current_position = np.array([dave.x, dave.y])
        target_vector = self.target_position - current_position  # type: ignore

        distance_error = float(np.linalg.norm(target_vector))
        target_direction_unit_vector = target_vector/distance_error

        current_direction_unit_vector = dave.get_front_facing_vector().flatten()

        orientation_error_magintude = np.arccos(
            np.dot(target_direction_unit_vector, current_direction_unit_vector)
        )
        orientation_error_direction = np.cross(
            target_direction_unit_vector,
            current_direction_unit_vector
        ).flatten()[0]
        orientation_error_direction = orientation_error_direction / \
            abs(orientation_error_direction)

        orientation_error = orientation_error_direction*orientation_error_magintude

        rotate_control_signal = self.pd_rotate.control(orientation_error)
        linear_control_signal = self.pd_linear.control(distance_error)

        # setting the required omega
        wheels_rotate_term = rotate_control_signal*self.rotate_max
        wheels_linear_term = 0

        absoulte_error = abs(self.pd_linear.get_error())
        # setting the required_linear_velocity
        if(abs(self.pd_rotate.get_error()) < first_turn_threshold):
            # only start linear motion if this is met
            wheels_linear_term = -linear_control_signal*self.w_max

        if(absoulte_error < linear_threshold):  # avoding small vector errors
            wheels_rotate_term = 0
        left_wheel_velocity = wheels_linear_term-wheels_rotate_term
        right_wheel_velocity = wheels_linear_term+wheels_rotate_term
        dave.set_velcoity(left_wheel_velocity, right_wheel_velocity)

        # print(f"{current_direction_unit_vector=}")
        # print(f"{target_direction_unit_vector=}")
        # print(f"{orientation_error}")
        # print(f"{self.pd_rotate.get_error()=}")
        # print(f"{wheels_rotate_term=}")
        # print(f"{left_wheel_velocity=}, {right_wheel_velocity=}")
        # # print(f"{self.pd_linear.get_error()=}")
        # print("******************************")

        if(absoulte_error < linear_threshold):
            return True

        return False
