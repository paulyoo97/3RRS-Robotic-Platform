### Servo Control
from adafruit_servokit import ServoKit
import time

class motors:
    def __init__(self):
        # Angle Offset
        self.offset = 30

        #Initialize Servo Driver
        self.kit = ServoKit(channels=16)

        # Initialize servo set up
        self.servo1 = self.kit.servo[0]
        self.servo2 = self.kit.servo[2]
        self.servo3 = self.kit.servo[4]

        # Initialize PWM range
        self.servo1.set_pulse_width_range(570,2750)
        self.servo2.set_pulse_width_range(570,2750)
        self.servo3.set_pulse_width_range(570,2750)
        time.sleep(2)   # Delay before utilization

    def spin(self, alpha):
        self.servo1.angle = int(alpha[0] + self.offset)
        self.servo2.angle = int(alpha[1] + self.offset)
        self.servo3. angle = int(alpha[2] + self.offset)
        time.sleep(0.01)