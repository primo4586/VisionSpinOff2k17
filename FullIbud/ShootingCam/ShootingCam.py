import sys
sys.path.append("/home/pi/opencv-3.1.0/build/lib")
import cv2
from networktables import NetworkTables
import numpy as np
import math

class ShootingCam: #The shooting cam class
    def __init__(self): #The ctor
        self.defineGlobalVars() #Defines the global vars
        self.sliders() #Sets the sliders to choose the HSV
        self.cap = cv2.VideoCapture("http://root:root@10.45.86.11/mjpg/video.mjpg") #connects to the camera (11 is the shooting cam)
        self.currentCap=None #checks the current Cap
    def defineGlobalVars(self): #Defines the global vars
        '''counter strike''';global offensive #CS:GO
        self.sliderImg = None #The slider IMG
        self.shootHorFov = 67 #the Shoot horizontal Field Of View
        self.shootTanHorFov = math.tan(math.radians(self.shootHorFov / 2))#the Shoot tam horizontal Field Of View
        self.shootVerFov = 36.13 #The shooting cam Ver FOV
        self.tilt = 39.77 #the tilt of the camera (angle) from the surface
        self.img = None #sets the img to none
        self.res = None #sets the res to none
        self.mask = None #sets the mask to none
        self.contours = None #sets the contours to none
        self.horizontalAngle = 0 #sets the horizontal angle to 0
        self.prevHorAngle = 0 #sets the previous horizontal angle to 0
        self.horDistance = 0 #sets the horizontal distance to 0
        self.prevHorDistance = 0 #sets the previous horizontal distance to 0
        self.isPrevRetroFound = False #checks if "retro" has been printed
        self.imgHeight = None #sets the img Height to none
        self.imgWidth = None #sets the img width to none
        self.retroHeight = 218.44 #7 feet and 2 inches to cm
        self.cameraHeight = 59.5 #sets the camera height (from the surface)

    def nothing(self,_): #something to do with the sliders
        pass #Passes

    def getAvgMidX(self, con1, con2): #returns the avg mid X of two contours
        x1, y1, w1, h1 = cv2.boundingRect(con1) #Gets the corners of the rectangle of the first contour
        x2, y2, w2, h2 = cv2.boundingRect(con2) #Gets the corners of the rectangle of the second contour
        midX = (x1 + w1 / 2 + x2 + w2 / 2) / 2 #doing the avg of the two rectangles
        cv2.rectangle(self.img, (x1, y1), (x1 + w1, y1 + h1), (0, 100, 0), 2) #draws the rectangle of the first contour
        cv2.rectangle(self.img, (x2, y2), (x2 + w2, y2 + h2), (0, 100, 0), 2) #draws the rectangle of the second contour
        return midX #returns the avg mid x

    def getMidX(self, con): #returns mid of one contour (the center)
        x, y, w, h = cv2.boundingRect(con) #gets the corners of the contour
        midX = x + w / 2 #doing the avg between the first corner and the width
        cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 100, 0), 2) #draws the rectangle based on the contour
        return midX #returns the avg mid x

    def getAvgMidXY(self, con1, con2): #returns the avg mid X and the mid Y of two contours
        x1, y1, w1, h1 = cv2.boundingRect(con1) #Gets the corners of the rectangle of the first contour
        x2, y2, w2, h2 = cv2.boundingRect(con2) #Gets the corners of the rectangle of the second contour
        midX = (x1 + w1 / 2 + x2 + w2 / 2) / 2 #doing the avg of the two rectangles (the x value)
        midY = (y1 + h1 / 2 + y2 + h2 / 2) / 2 #doing the avg of two rectangles (the y value)
        cv2.rectangle(self.img, (x1, y1), (x1 + w1, y1 + h1), (50, 100, 50), 1) #draws the first rectangle
        cv2.rectangle(self.img, (x2, y2), (x2 + w2, y2 + h2), (50, 100, 50), 1) #draws the second rectangle
        return midX, midY #returns both x value and y value

    def getMidXY(self, con): #returns the mid X and mid Y for a contour
        x, y, w, h = cv2.boundingRect(con) #gets the corners of a contour
        midX = x + w / 2 #the middle of the contour (x wise)
        midY = y + h / 2 #the middle of the contour (y wise)
        cv2.rectangle(self.img, (x, y), (x + w, y + h), (0, 255, 0), 2) #draws a rectangle of a contour
        return midX, midY #returns both x value and y value of the middle

    #Creates a golbal slider image and adds sliders to it
    def sliders(self):
        self.sliderImg = np.zeros((1, 300, 3), np.uint8) #sets an sliders variable
        cv2.namedWindow('slider') #prints a window named slider
        cv2.createTrackbar('Hmin', 'slider', 0, 255, self.nothing) #creates a HSV type bar
        cv2.createTrackbar('Hmax', 'slider', 255, 255, self.nothing) #creates a HSV type bar
        cv2.createTrackbar('Smin', 'slider', 0, 255, self.nothing) #creates a HSV type bar
        cv2.createTrackbar('Smax', 'slider', 255, 255, self.nothing) #creates a HSV type bar
        cv2.createTrackbar('Vmin', 'slider', 0, 255, self.nothing) #creates a HSV type bar
        cv2.createTrackbar('Vmax', 'slider', 255, 255, self.nothing) #creates a HSV type bar
        cv2.createTrackbar('distanceMult', 'slider', 185, 200, self.nothing) #creates a HSV type bar



    def getImage(self): #returns the image/res/mask
        _,self.img = self.cap.read() #gets the image from the cap we defined earlier
        if (self.img is None): #if the image is none
            self.cap = cv2.VideoCapture("http://root:root@10.45.86.11/mjpg/video.mjpg") #reconnect to the camera
            _, self.img = self.cap.read() #re-get the image
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV) #makes the hsv of the img
        #lower_green = np.array([cv2.getTrackbarPos('Hmin','slider'), cv2.getTrackbarPos('Smin','slider'), cv2.getTrackbarPos('Vmin','slider')]) #lower values of the hsv
        #higher_green = np.array([cv2.getTrackbarPos('Hmax','slider'), cv2.getTrackbarPos('Smax','slider'), cv2.getTrackbarPos('Vmax','slider')]) #higher values of the hsv
        lower_green = np.array([14, 66, 58])
        higher_green = np.array([136, 255, 255]) #Shooting 74, 58, 142 - 94, 255, 255
        #40, 141, 78
        #78,255,193 - Gear Sliders
        self.mask = cv2.inRange(hsv, lower_green, higher_green) #makes the mask of the img
        self.res = cv2.bitwise_and(self.img, self.img, mask = self.mask) #makes the res of the img
        _, self.contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #gets the contours from the mask
        return self.contours, self.img #returns the contours and the img (to main)

    def calcShootDistance(self, midY): #calc shooting distance
        fromCenter = (self.imgHeight * math.cos(math.radians(self.shootVerFov - self.tilt))) / math.sin(math.radians(self.shootVerFov)) #gets the distance from center
        tanVerticalAngle = (((0.5*self.imgHeight-midY) / fromCenter) + math.sin(math.radians(self.tilt))) / math.cos(math.radians(self.tilt)) #gets the vertical angle
        self.horDistance = ((self.retroHeight-self.cameraHeight) / tanVerticalAngle) #calculates the horizontal distance

    def calcShootHorizontalAngle(self, midX): #returns the horizontal angle of the shooting camera
        self.horizontalAngle = math.degrees(
            math.atan(((midX - 0.5 * self.imgWidth) * self.shootTanHorFov) / (0.5 * self.imgWidth))) #calculates the horizontal angle from the contours



    def shootCam(self, table): #The proccesing operation
        self.getImage() #gets the img
        self.imgHeight, self.imgWidth, _ = self.img.shape #gets the img height and width
        biggestContour = None #sets the biggest contour as None
        biggestArea = 0 #sets the biggest area as 0
        secondBiggestContour = None #sets the second biggest contour as none
        secondBiggestArea = 0 #sets the second biggest area as 0
        for con in self.contours: #enters the for loop based on the amount of contours found
            area = cv2.contourArea(con) #sets the area based on the current contour
            if (area > 100): #if the contour area is bigger than 100
                if (biggestArea is 0): #if the biggest area is 0 (enters first time)
                    biggestContour = con #sets the contour as the current contour
                    biggestArea = area #sets the biggest area as the current area
                elif (area > biggestArea): #if the area is bigger than the current biggest area
                    secondBiggestArea = biggestArea #sets the second biggest area to the first one
                    secondBiggestContour = biggestContour #same as the contour
                    biggestContour = con #sets the new biggest contour as the current contour
                    biggestArea = area #sets the new biggest area as the current contour
                elif (area > secondBiggestArea): #if the area is bigger than the second biggest area but not bigger than the first
                    secondBiggestArea = area #set the second biggest area as the current area
                    secondBiggestContour = con #set the second biggest contour as the current contour
        if (biggestContour is not None): #if the biggest contour is not none
            self.isPrevRetroFound = False #sets the retro to none
            if (secondBiggestContour is not None): #if the second biggest contour isn't none
                avgMidX, avgMidY = self.getAvgMidXY(biggestContour, secondBiggestContour) #gets the avg mid X and mid Y
                self.calcShootHorizontalAngle(avgMidX) #calcs the shooting horizontal angle
                self.calcShootDistance(avgMidY) #calcs the shooting distance
                if (abs(self.prevHorAngle - self.horizontalAngle) > 5): #if there is a differnce of over 20 degrees from the last angle
                    self.prevHorAngle = self.horizontalAngle #set the prev horangle to the current one
                    print "angle Shoot: ", self.horizontalAngle #print the angle
                    table.putValue("angleShoot", self.horizontalAngle) #return the angle to the table
                if (abs(self.prevHorDistance - self.horDistance) > 10): #if there is a differnce of over 10 cm from the last distance
                    self.prevHorDistance = self.horDistance #sets the prev distance to the current one
                    print "distance Shoot: ", self.horDistance #prints the current distance
                    table.putValue("distanceShoot", self.horDistance) #returns the distance to the table
            elif (secondBiggestContour is None): #if the second contour is none
                midX, midY = self.getMidXY(biggestContour) #gets the midX and midY of the biggest contour
                self.calcShootHorizontalAngle(midX) #calcs the horizontal angle
                self.calcShootDistance(midY) #calcs the distance
                if (abs(self.prevHorAngle - self.horizontalAngle) > 5): #if there is a differnce of over 20 degrees from the last angle
                    self.prevHorAngle = self.horizontalAngle #sets the prev hor angle to the current one
                    print "angle Shoot: ", self.horizontalAngle #prints the current hor angle
                    table.putValue("angleShoot", self.horizontalAngle) #returns the current hor angle to the table
                if (abs(self.prevHorDistance - self.horDistance) > 10): #if there is a differnce of over 10 cm from the last distance
                    self.prevHorDistance = self.horDistance #sets the prev hor distance to the current one
                    print "distance Shoot: ", self.horDistance #prints the current hor distance
                    table.putValue("distanceShoot", self.horDistance) #returns the hor distance to the table
        else:
            if (self.isPrevRetroFound is False): #check if "no retro found" has already been printed
                self.isPrevRetroFound = True #set as true
                print "no retro found" #prints no retro found
