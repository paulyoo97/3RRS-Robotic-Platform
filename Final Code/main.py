# Libraries
import numpy as np
import time
import cv2

# Import Classes
import sensor
import PID
import inverseKinetmatics
import actuator

# Initial frame
frame = np.zeros((480, 480, 3), dtype=np.uint8)

K = [0.33,0.011,0.235]
origin = [0,0]

# Create objects from class
camera = sensor.Camera()
calculate = inverseKinetmatics.calculations()
move = actuator.motors()
pid = PID.controller(K)

# Prepare live camera frame and position
x=y=-1
location = [x,y]
# Initial position
move.spin([0,0,0])
time.sleep(1)

def live():
    global x,y
    while True:
        x,y = camera.position()
        location = [x,y]
        if x !=0:
            phi,theta = pid.compute(origin, location)
            psi = 0
            b,p,q = calculate.joints(phi,theta,psi)
            l = calculate.effect_leg(b,p,phi,theta,psi)
            alpha = calculate.motor_angles(b,q,l)
            move.spin(alpha)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    camera.close_camera()
    time.sleep(1)
    move.spin([60,60,60])
    time.sleep(1)
    move.spin([-30,-30,-30])
    print('Program is Complete')

live()
