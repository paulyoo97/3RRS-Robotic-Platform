# PID Controller
import math
import time
from scipy.interpolate import interp1d

class controller:
    def __init__(self,k):   # k is an array with gain values
        # the PID gains
        self.kp = k[0]
        self.ki = k[1]
        self.kd = k[2]

        # x and y output limits
        self.min_value = -180 # change
        self.max_value = 180 # change

        # phi and theta rotation limits in degrees
        self.min_angle = -10
        self.max_angle = 10

        # Initial values
        # previous output values
        self.previous_x_out = 0
        self.previous_y_out = 0

        # previous errors
        self.previous_x_error = 0
        self.previous_y_error = 0

        # Integral values
        self.x_integral = 0
        self.y_integral = 0

        # Time
        self.last_time = None


    def compute(self, origin, location):
        current_time = time.perf_counter()
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0

        # Get the error
        x_error = (origin[0] - location[0])
        y_error = (origin[1] - location[1])

        # Integral
        self.x_integral += x_error*((current_time - self.last_time))
        self.y_integral += y_error*((current_time - self.last_time))

        # Derivative
        x_derivative = (x_error - self.previous_x_error)/((current_time - self.last_time))
        y_derivative = (y_error - self.previous_y_error)/((current_time - self.last_time))

        # Output
        x_out = self.kp*x_error + self.ki*self.x_integral + self.kd*x_derivative
        y_out = self.kp*y_error + self.ki*self.y_integral + self.kd*y_derivative

        if abs(x_out) > 150:
            x_out = x_out/abs(x_out)*150

        if abs(y_out) > 150:
            y_out = y_out/abs(y_out)*150

        # interpolate for phi and theta
        # create interpolation
        find_angle = interp1d([self.min_value, self.max_value],[self.min_angle, self.max_angle])
        
        # Get the angles
        phi = -1*find_angle(y_out) # given y output, we can get rotation about x
        theta = find_angle(x_out) # given x output, we can get the rotation about y
              
        # Assign updated previous values
        self.previous_x_error = x_error
        self.previous_y_error = y_error
        self.previous_x_out = x_out
        self.previous_y_out = y_out
        self.last_time = current_time
        # print(f'Errors: {x_error,y_error}')
        # print(f'PID output: {x_out,y_out}')
        # print(f'Platform angles:{phi,theta}')
        return phi,theta # required rotation about x and y axis, respectively.
    