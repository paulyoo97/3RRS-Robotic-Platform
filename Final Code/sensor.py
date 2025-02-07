from picamera2 import Picamera2
import cv2
import numpy as np

class Camera:
    def __init__(self):
        # Camera Object
        self.width = 480
        self.height = 480
        self.picam2 = Picamera2()

        # Configure the Camera
        self.configuration = self.picam2.create_video_configuration(
            raw={'size':(1640,1232)},   # raw is for the incoming feed.
            main={'format':'RGB888','size':(self.width,self.height)})
        self.picam2.configure(self.configuration)

        # Green in HSV color model
        self.lower_green = np.array([29, 86, 6])
        self.upper_green = np.array([102, 255, 255])
        self.picam2.start()

    # Get the x and y position of the ball used for PID algorithm
    def position(self):
        frame = self.picam2.capture_array()
        img = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) # Converts from BGR to HSV color model
        mask = cv2.inRange(img,self.lower_green,self.upper_green)   # Masks the colors        
        # Erode and dilate
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)
        
        contours, hiearchy = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Gets the contour based on the mask.
        if contours:
            largest_contour = max(contours, key = cv2.contourArea) # uses the function in key to get the contour with the largest area.
            (x,y),radius = cv2.minEnclosingCircle(largest_contour)
            area = cv2.contourArea(largest_contour)
            if area>200:
                cv2.circle(frame,(int(x),int(y)),int(radius),(0,0,255),2)  # contour will have a width of 2
                cv2.imshow('Output',frame)
                cv2.waitKey(1)
                x -= 255
                y -= 247
                x,y = -y,x  # Required coorindates
                # print(f'x: {int(x)}, y: {int(y)}')
                return x, y
        cv2.imshow('Output',frame)
        cv2.waitKey(1)
        return 0,0
    
    def close_camera(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()