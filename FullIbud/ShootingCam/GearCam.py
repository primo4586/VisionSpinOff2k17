import sys
sys.path.append("/home/pi/opencv-3.1.0/build/lib")
import cv2
from networktables import NetworkTables
import numpy as np
import math

class Gear: #The gear cam class
    def __init__(self): #the ctor
        self.defineGlobalVars() #defines the global vars
        self.sliders() #stats the sliders
        self.cap = cv2.VideoCapture("http://root:root@10.45.86.12/mjpg/video.mjpg") #connects to the camera (12 is the shooting cam)
        self.currentCap = None #sets the current cap to None
    def defineGlobalVars(self): #defines the global vars
        '''counter strike''';global offensive #CS:GO
        self.slider = None #sets the sliders to None
        self.gearFov = 47 #sets the gear fov to 47
        self.gearTanFov = math.tan(math.radians(self.gearFov / 2)) #sets the tan fov
        self.contours = None #sets the contours as None
        self.img = None #sets the img as none
        self.res = None #sets the res as none
        self.mask = None #sets the mask as none
        self.contours = None #sets the contours as none
        self.horizontalAngle = 0 #sets the horizontal angle as 0
        self.prevHorAngle = 0 #sets the prev horizontal angle as 0
        self.horDistance = 0 #sets the horizontal distance as 0
        self.prevHorDistance = 0 #sets the prev horizontal distance as 0
        self.isPrevRetroFound = False #sets if retro has been printed as False
        self.imgHeight = None #sets the img height as None
        self.imgWidth = None #sets the img width as None
        self.gearHeightIrl = 12.7 #5 inch to cm
        self.gearWidthIrl = 5.08 #2 inch to cm
        self.retroHeight = 218.44 #7 feet and 2 inches to cm
        self.cameraHeight = 59.5 #sets the camera height from the surface

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

    # Creates a golbal slider image and adds sliders to it
    def sliders(self):
        self.sliderImg = np.zeros((1, 300, 3), np.uint8)  # sets an sliders variable
        cv2.namedWindow('slider')  # prints a window named slider
        cv2.createTrackbar('Hmin', 'slider', 0, 255, self.nothing)  # creates a HSV type bar
        cv2.createTrackbar('Hmax', 'slider', 255, 255, self.nothing)  # creates a HSV type bar
        cv2.createTrackbar('Smin', 'slider', 0, 255, self.nothing)  # creates a HSV type bar
        cv2.createTrackbar('Smax', 'slider', 255, 255, self.nothing)  # creates a HSV type bar
        cv2.createTrackbar('Vmin', 'slider', 0, 255, self.nothing)  # creates a HSV type bar
        cv2.createTrackbar('Vmax', 'slider', 255, 255, self.nothing)  # creates a HSV type bar
        cv2.createTrackbar('distanceMult', 'slider', 185, 200, self.nothing)  # creates a HSV type bar

    def getImage(self):  # returns the image/res/mask
        _, self.img = self.cap.read()  # gets the image from the cap we defined earlier
        if (self.img is None):  # if the image is none
            self.cap = cv2.VideoCapture("http://root:root@10.45.86.12/mjpg/video.mjpg")  # reconnect to the camera
            _, self.img = self.cap.read()  # re-gets the image
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)  # makes the hsv of the img
        #lower_green = np.array([cv2.getTrackbarPos('Hmin', 'slider'), cv2.getTrackbarPos('Smin', 'slider'),
                                #cv2.getTrackbarPos('Vmin', 'slider')])  # lower values of the hsv
        #higher_green = np.array([cv2.getTrackbarPos('Hmax', 'slider'), cv2.getTrackbarPos('Smax', 'slider'),
                                 #cv2.getTrackbarPos('Vmax', 'slider')])  # higher values of the hsv
        lower_green = np.array([14, 66, 58])
        higher_green = np.array([136, 255, 255]) #Shooting 74, 58, 142 - 94, 255, 255
        # 40, 141, 78
        # 78,255,193 - Gear Sliders
        self.mask = cv2.inRange(hsv, lower_green, higher_green)  # makes the mask of the img
        self.res = cv2.bitwise_and(self.img, self.img, mask=self.mask)  # makes the res of the img
        _, self.contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)  # gets the contours from the mask
        return self.contours, self.img  # returns the contours and the img (to main)


    def calcGearHorizontalAngle(self, midX): #calcs the gear cam horizontal angle from the contour
        self.horizontalAngle = math.degrees(
            math.atan(((midX - 0.5 * self.imgWidth) * self.gearTanFov) / (0.5 * self.imgWidth))) #the equation

    def calcGearDistance(self, biggestContour, midX): #calcs the gear cam horizontal distance
        perpendicular = 0.5 * (self.imgWidth / self.gearTanFov) #the perpendicular
        _, _, w, h = cv2.boundingRect(biggestContour) #gets the width and height of the biggest contour
        ratio = self.gearHeightIrl / h #calcs the ratio
        #Calcs the distance to the middle of the 2 countours
        self.horDistance = ratio * math.sqrt(math.pow((midX - 0.5 * self.imgWidth), 2) + math.pow(perpendicular, 2))
        #If needed the perpendicular distance the formula is: self.horDistance = ratio * perpendicular

    def gearCam(self, table): #the gear cam operation
        self.getImage() #Gets the img
        self.imgHeight, self.imgWidth, _ = self.img.shape #gets the img height and width
        # sorts the two biggest contours based on area
        biggestContour = None #sets the biggest contour as none
        area = 0 #sets the area as 0
        biggestArea = 0 #sets the biggest area as 0
        secondBiggestContour = None #sets the second biggest contour as None
        secondBiggestArea = 0 #sets the second biggest area as 0
        for con in self.contours: #starts the loop of all the contours
            area = cv2.contourArea(con) #sets the area of the current contour
            if (area > 100): #if the current area is bigger than 100
                if (biggestArea is 0): #if biggest area is 0 (enters once)
                    biggestContour = con #sets the biggest contour as the current contour
                    biggestArea = area #sets the biggest area as the current area
                elif (area > biggestArea): #if the current area is bigger than the biggest area
                    secondBiggestArea = biggestArea #sets the second biggest area as the last biggest area
                    secondBiggestContour = biggestContour #sets the second biggest contour as the last biggest contour
                    biggestContour = con #sets the biggest contour as the current contour
                    biggestArea = area #sets the biggest area as the current area
                elif (area > secondBiggestArea): #if area is bigger than the second biggest area but not the biggest one
                    secondBiggestArea = area #sets the second biggest area as the current area
                    secondBiggestContour = con #sets the second biggest contour as the current contour
        if (biggestContour is not None): #if the biggest contour is not None
            self.isPrevRetroFound = False #defines the "is retro has been printed" as false
            if (secondBiggestContour is not None): #if the second biggest contour is not none
                avgMidX = self.getAvgMidX(biggestContour, secondBiggestContour) #gets the avg mid x of the biggest contour and the second biggest contour
                self.calcGearHorizontalAngle(avgMidX) #calcs the horizontal angle of the two contours
                self.calcGearDistance(biggestContour, avgMidX) #calcs the distance from the gear contours based on the biggest contour
                if (abs(self.prevHorAngle - self.horizontalAngle) > 0): #if the differnce between the last angle and the current angle is over 5
                    self.prevHorAngle = self.horizontalAngle #sets the prev horizontal angle as the current angle
                    print "angle Gear: ", self.horizontalAngle #prints the angle gear
                    if (self.horizontalAngle > 0):
                        table.putValue("angleGear",
                                       self.horizontalAngle - 20)  # returns the horizontal angle to the table
                    else:
                        table.putValue("angleGear",
                                       self.horizontalAngle + 20)  # returns the horizontal angle to the table
                if (abs(self.prevHorDistance - self.horDistance) > 0): #if the differnce between the last distance and the current distnace is over 10
                    self.prevHorDistance = self.horDistance #sets the prev horizontal distance as the current one
                    print "distance Gear: ", self.horDistance #prints the current distance
                    table.putValue("distanceGear", self.horDistance) #returns the horizontal distance to the table
            elif (secondBiggestContour is None): #if the second biggest contour is none
                midX = self.getMidX(biggestContour) #gets the midx of the biggest contour
                self.calcGearHorizontalAngle(midX) #calcs the gear horizontal angle
                self.calcGearDistance(biggestContour, midX) #calcs the gear horizontal distance
                if (abs(self.prevHorAngle - self.horizontalAngle) > 0): #if the differnce between the last angle and the current angle is over 5
                    self.prevHorAngle = self.horizontalAngle #sets the prev horizontal angle as the current horizontal angle
                    print "angle Gear: ", self.horizontalAngle #prints the angle
                if (self.horizontalAngle > 0):
                    table.putValue("angleGear", self.horizontalAngle - 20) #returns the horizontal angle to the table
                else:
                    table.putValue("angleGear", self.horizontalAngle + 20)  # returns the horizontal angle to the table
                if (abs(self.prevHorDistance - self.horDistance) > 0): #if the differnce between the last distance and the current distnace is over 10
                    self.prevHorDistance = self.horDistance #sets the prev hor distance as the current distance
                    print "distance Gear: ", self.horDistance #prints the current distance
                    table.putValue("distanceGear", self.horDistance) #returns the horizontal distance to the table
        else:
            if (self.isPrevRetroFound is False): #if the "is retro printed" is false
                self.isPrevRetroFound = True #sets the prev retro to true
                print "no retro found" #prints no retro found

