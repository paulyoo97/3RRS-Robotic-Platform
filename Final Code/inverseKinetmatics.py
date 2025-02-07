# Inverse Kinematics
from scipy.spatial.transform import Rotation
import numpy as np
import math

class calculations:
    def __init__(self):
        # Constants
        # lengths measured in mm
        # angles measured in degrees

        self.a = 53  # length of servo arms
        self.s = 150.5 # length of operating arms
        self.h = 148.4 # distance from center of platform relative to center of base (148.483 mm is actual value)
        self.rb = 65 # base radius
        self.rp = 91.2 # platform radius
        self.B = [0,120,240] # angles for each servo arm plane relative to x-axis, shaft axis lies on XY plane where z=0
        self.anchor_orientation = self.B   # used for identifying point of rotation of servos and platform spherical joints

    # Rotation Matrix about the z-axis
    # this will be used to identify the ith location of servo and spherical joint relative to base
    def rotz(self,angle):
        r = Rotation.from_euler('z',angle,degrees=True)
        return r.as_matrix()

    # Rotation Matrix
    # this will be used to later calculate the spherical joint and effective leg vectors
    def get_Rotation(self,phi,theta,psi):   # rotation about x,y, and z used later for platform orientation
        rx = Rotation.from_euler('x',phi,degrees=True).as_matrix()
        ry = Rotation.from_euler('y',theta,degrees=True).as_matrix()
        rz = Rotation.from_euler('z',psi,degrees=True).as_matrix()
        rotated = rz@ry@rx
        return rotated
    
    # Joint Locations
    def joints(self,phi,theta,psi):
        # matrix for anchors
        b = np.zeros((3,3)) # empty base anchor matrix array (point of rotation of servos)
        p = np.zeros((3,3)) # empty platform anchor matrix array (spherical joints)

        # compute anchor matrix arrays
        # columns of matrix are 
        for i in range(3):
            # convention to change column in a matrix array is to provide a row
            b[:,i] = (self.rotz(self.anchor_orientation[i])@[[self.rb],[0],[0]]).flatten()
            p[:,i] = (self.rotz(self.anchor_orientation[i])@[[self.rp],[0],[0]]).flatten()

        # column translation vector
        T = [[0],[0],[self.h]]

        # spherical joint relative to reference frame
        q = np.zeros((3,3))
        for i in range(3):
            q[:,i] = (T + self.get_Rotation(phi,theta,psi)@(p[:,i].reshape(-1,1))).flatten()
        
        return b,p,q
    
    # Effective Leg
    def effect_leg(self,b,p,phi,theta,psi):
        l = np.zeros((3,3))
        T = [[0],[0],[self.h]]
        for i in range(3):
            l[:,i] = (T + self.get_Rotation(phi,theta,psi)@(p[:,i].reshape(-1,1)) - b[:,i].reshape(-1,1)).flatten()
        return l
    
    # Required Servo Angles
    def motor_angles(self,b,q,l):
        alpha = np.zeros(3)
        for i in range(3):
            length = np.linalg.norm(l[:,i])

            xq = q[0,i]
            xb = b[0,i]

            yq = q[1,i]
            yb = b[1,i]

            zq = q[2,i]
            zb = b[2,i]

            L = length**2 - (self.s**2 - self.a**2)
            M = 2*self.a*(zq - zb)
            N = 2*self.a*(np.cos(np.radians(self.B[i]))*(xq - xb) + np.sin(np.radians(self.B[i]))*(yq - yb))
            alpha[i] = int(np.rad2deg(np.arcsin(L/math.sqrt(M**2 + N**2)) - np.arctan(N/M)))
        return alpha  