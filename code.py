import RPi.GPIO as GPIO
import random
from gpiozero import DistanceSensor
from threading import Thread
import time
import cv2
from picamera import PiCamera
GPIO.setmode(GPIO.BCM)
camera = PiCamera()
camera.rotation = 180
 
class Car(Thread):
    def __init__(self, left1, left2, left3, left4, right1, right2, right3, right4, avg_x, octagon, already_octagon):
        super().__init__()
        self.left1 = left1
        self.left2 = left2
        self.left3 = left3
        self.left4 = left4
        self.right1 = right1
        self.right2 = right2
        self.right3 = right3
        self.right4 = right4
        self.avg_x = avg_x
        self.octagon = octagon
        self.already_octagon = already_octagon
       
        self.pins = [left1, left2, left3, left4, right1, right2, right3, right4]
       
        self.turn_left_pins = [right1, right2, right3, right4, left4, left3, left2, left1]
        self.turn_right_pins = [left1, left2, left3, left4, right4, right3, right2, right1]
       
       
        for i in self.pins:
            GPIO.setup(i, GPIO.OUT)
           
    def run(self):
       
        while True:
            camera.capture('/home/pi/Desktop/original.jpg')
            pic = cv2.imread('/home/pi/Desktop/original.jpg')
            pic2 = pic
 
            #Color identification
            hsv = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
            blue_hsv = cv2.inRange(hsv, (0), (70))
            cv2.imwrite('/home/pi/Desktop/blue_hsv.jpg', blue_hsv)
 
            hsv = cv2.cvtColor(pic2, cv2.COLOR_BGR2HSV)
            red_hsv = cv2.inRange(hsv, (165, 0, 0), (172, 255, 255))
            cv2.imwrite('/home/pi/Desktop/red_hsv.jpg', red_hsv)
 
            #Cropping image
            (height, width) = blue_hsv.shape
            blue_hsv = blue_hsv[int(height/2):height, :]
            pic = pic[int(height/2):height, :]
            cv2.imwrite('/home/pi/Desktop/cropped.jpg', blue_hsv)
           
 
            #Edge Identification
            edges = cv2.Canny(blue_hsv, 100, 200)
            cv2.imwrite('/home/pi/Desktop/edges.jpg', edges)
            edges2 = cv2.Canny(red_hsv, 100, 200)
            cv2.imwrite('/home/pi/Desktop/edges2.jpg', edges2)
 
            #Line Identification
            rho = 1
            angle = 3.1415 / 180
            min_threshold = 10 
            line_segments = cv2.HoughLinesP(edges, rho, angle, min_threshold, None, minLineLength=15, maxLineGap=4)
 
 
            #Drawing detected lines onto image
            if line_segments is not None:
                xsum = 0
                count = 0
                for i in range(0, len(line_segments)):
                    line = line_segments[i][0]
                    cv2.line(pic, (line[0], line[1]),  (line[2], line[3]), (0, 0, 255), 3, cv2.LINE_AA)
                    x_coord_mid = (line[0]+line[2])/2
                    xsum += x_coord_mid
                    count += 1
                cv2.imwrite('/home/pi/Desktop/lines.jpg', pic)
 
            self.avg_x = xsum/count
           
 
 
            contours, hierarchy = cv2.findContours(edges2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
 
            octo_counter = 0
            if contours is not None:
                for shape in contours:
                    area = cv2.contourArea(shape)
                    approx = cv2.approxPolyDP(shape, 5, True)
                    if len(approx) == 8 and area > 1500:
                        cv2.drawContours(pic2, [shape], 0, (0, 255, 0), 2)
                        print('Octagon approaching! Area: ', area)
                        octo_counter += 1
                        self.octagon = True
                if octo_counter == 0:
                    self.octagon = False
                    '''
                    if len(approx) == 8 and area < 6000 and area > 4000:
                        cv2.drawContours(pic, [shape], 0, (0, 255, 0), 2)
                        print('Octagon detected! Area: ', area)
                        self.octagon = True
                    '''
                cv2.imwrite('/home/pi/Desktop/final.jpg', pic2)
           
    def both_forward(self, distance):
        for i in range(distance):
            for pin in range(4):
                GPIO.output(self.left_wheel_forward[pin], 1)
                GPIO.output(self.right_wheel_forward[pin], 1)
                time.sleep(0.003)
                GPIO.output(self.left_wheel_forward[pin], 0)
                GPIO.output(self.right_wheel_forward[pin], 0)
               
    def both_backward(self, distance):
        for i in range(distance):
            for pin in range(3,-1,-1):
                GPIO.output(self.left_wheel_forward[pin], 1)
                GPIO.output(self.right_wheel_forward[pin], 1)
                time.sleep(0.003)
                GPIO.output(self.left_wheel_forward[pin], 0)
                GPIO.output(self.right_wheel_forward[pin], 0)
   
    def turn_left(self, distance):
        for i in range(distance):
            for pin in self.turn_left_pins:
                GPIO.output(pin, 1)
                time.sleep(.004)
                GPIO.output(pin, 0)
       
               
    def turn_right(self, distance):
        for i in range(distance):
            for pin in self.turn_right_pins:
                GPIO.output(pin, 1)
                time.sleep(.004)
                GPIO.output(pin, 0)
 
       
    def wheel_off(self):
        GPIO.output(self.left1, 0)
        GPIO.output(self.left2, 0)
        GPIO.output(self.left3, 0)
        GPIO.output(self.left4, 0)
        GPIO.output(self.right1, 0)
        GPIO.output(self.right2, 0)
        GPIO.output(self.right3, 0)
        GPIO.output(self.right4, 0)
       
    def motor_control(self):
        backwards_check = 0
        lcd_screen = None
        while True:
            if self.octagon == True and self.already_octagon == False:
                self.wheel_off()
                time.sleep(5)
                self.already_octagon = True
            elif self.octagon == False and self.already_octagon == True:
                self.already_octagon = False
            if self.avg_x <= 342:
                self.turn_left(5)
            elif self.avg_x >= 682:
                self.turn_right(5)
            else:
                self.both_forward(5)
 
robot = Car(14, 4, 17, 27, 16, 26, 20, 21, 0, False, False)
 
robot.start()
robot.motor_control()
