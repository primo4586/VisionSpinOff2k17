import math
import sys
from pyasn1.compat.octets import null
sys.path.append("/home/pi/opencv-3.1.0/build/lib")
import cv2
import numpy as np
from networktables import NetworkTables
import sys
import time
#basad
#Making The
#distance calc with angle
def printNoRetroFound(x,counter3):
    if (counter3 is 0):
        print "no retro found"
def nothing(self):
    pass
def printAngle(angle, counter2):
    if (counter2 is 1):
        print "angle:", angle
def calcDistance(img, width, fov, widthIrl):
    imgHieght, imgWidth, _ = img.shape
    distance = (widthIrl / 2) * (imgHieght) / (width * math.tan(math.radians(fov) / 2))
    distance *= 4.546
    #distance *= cv2.getTrackbarPos('distanceMult', 'slider') / 100.0
    return abs(distance)
def makeHSV(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #lower_green = np.array([cv2.getTrackbarPos('Hmin','slider'), cv2.getTrackbarPos('Smin','slider'), cv2.getTrackbarPos('Vmin','slider')])
    #higher_green = np.array([cv2.getTrackbarPos('Hmax','slider'), cv2.getTrackbarPos('Smax','slider'), cv2.getTrackbarPos('Vmax','slider')])
    lower_green = np.array([40,141,78])
    higher_green = np.array([78,255,193])
    mask = cv2.inRange(hsv, lower_green, higher_green)
    res = cv2.bitwise_and(img, img, mask=mask)
    return res
#note:find the contours
def findContours(img):
    im2, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours
#note: calculates angle from contours
def distanceCheck(img, width, fov, widthIrl, counter):
    if (counter is 1):
        imgHieght, imgWidth, _ = img.shape
        distance = (widthIrl / 2) * (imgHieght) / (width * math.tan(math.radians(fov) / 2))
        distance *= 4.546
        #distance *= cv2.getTrackbarPos('distanceMult', 'slider') / 100.0
        print "distance", distance
        return distance

def calcAngle(img,midX,tanFOV):
        imgHieght, imgWidth, _ = img.shape
        return math.degrees(math.atan(((imgWidth/2-midX)*2*tanFOV)/imgWidth))
def calcMid(b, d):
    y = (b[1] + d[1]) / 2
    x = (b[0] + d[0]) / 2
    return x, y
#note: Main
if __name__ == '__main__':
    reload(sys)
    #network table aka smartdashboard
    NetworkTables.setIPAddress("10.45.86.2")
    NetworkTables.setClientMode()
    NetworkTables.initialize()
    table = NetworkTables.getTable('imgProc')
    table.putValue("angle", 0)
    cap = cv2.VideoCapture("http://root:root@10.45.86.12/mjpg/video.mjpg")
    tanFov = math.tan(math.radians(47/2))
    sliderImg = np.zeros((1,300,3),np.uint8)
    cv2.namedWindow('slider')
    #sliders to determine the HSV
    cv2.createTrackbar('Hmin','slider',0,255,nothing)
    cv2.createTrackbar('Hmax','slider',255,255,nothing)
    cv2.createTrackbar('Smin','slider',0,255,nothing)
    cv2.createTrackbar('Smax','slider',255,255,nothing)
    cv2.createTrackbar('Vmin','slider',0,255,nothing)
    cv2.createTrackbar('Vmax','slider',255,255,nothing)
    cv2.createTrackbar('distanceMult','slider',185,200,nothing)
    angleCounter1 = 0
    angle1 = 0
    #both checks when to print the angle and the distance
    counter = 1
    counter2 = 1
    counter3 = 0
    #the prev distance to check if there are high jumps
    prevDistance = 0
    while True:
        #gets the image
        cv2.imshow('slider',sliderImg)
        ret, img = cap.read()
        cv2.imshow("uncut image",img)
        res = makeHSV(img)
        cv2.imshow('res',res)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        contours = findContours(gray)
        x = 0
        area1 = 0
        area2 = 0
        #loop that defines both contours by size
        for i in contours:
            area = cv2.contourArea(i)
            if area > 100:
                if x is 0:
                    con1 = i
                    area1 = area
                    con2 = None
                    x+= 1
                elif area > area1:
                    con2 = con1
                    area2 = area1
                    con1 = i
                    area1 = area
                elif area > area2:
                    con2 = i
                    area2 = area
        if(x!=0):
            #gets in if it finds both contours
            if (con2 != None and con1 != None):
                #gets the height and the width of both contours
                rect1 = cv2.minAreaRect(con1)
                rect2 = cv2.minAreaRect(con2)
                width1 = min(rect1[1])
                height1 = max(rect1[1])
                height2 = max(rect2[1])
                #Img Height and Img Width
                imgHieght, imgWidth, _ = img.shape
                #Contours Corners
                box1 = np.int0(cv2.boxPoints(rect1))
                box2 = np.int0(cv2.boxPoints(rect2))
                #prints the contours
                finalHeight = (height1 + height2) / 2
                cv2.drawContours(gray, [box1], 0, (255, 255, 255), 2)
                cv2.drawContours(gray, [box2], 0, (255, 255, 255), 2)
                #finds both middle points for both contours
                midX, midY=calcMid(b=box1[1],d=box1[3])
                midX1, midY2 = calcMid(b=box2[1], d=box2[3])
                #finds the middle between both middles
                topMidX = (midX + midX1) / 2
                topMidY = (midY + midY2) / 2
                #calculate the angle
                angle = calcAngle(img=img,midX=topMidX,tanFOV=tanFov)
                distance = calcDistance(img, finalHeight, 47, 5)
                #axis m1011 fov = 47
                if (abs(angle1 - angle) < 2):
                    distanceCheck(img, height1, 47, 5, counter)
                    printAngle(angle,counter)
                    counter3 = 0
                    counter = 0
                else:
                    angle1 = angle
                    counter = 1
                    counter3 = 0
                    #TODO - Add comments
                    if (abs(distance - prevDistance) > 5 and prevDistance != 0):
                        if (distance > prevDistance):
                            print "distance", distance
                            print "angle", angle
                            prevDistance = distance
                        else:
                            print "distance", prevDistance
                            print "angle", angle
                            # distance = prevDistance
                    else:
                        print "distance", distance
                        print "angle", angle
                        prevDistance = distance
                if (angle1 is 0):
                    angle1 = angle
                if (prevDistance is 0):
                    prevDistance = distance
                #prints the angle
                cv2.imshow("gray", gray)
                #returns angle to the robot
                table.putValue("angle", angle)
                table.putValue("distance", distance)
                #if there isn't two contours
                counter3 = 0
            if (con2 is None and con1 != None):
                rect1 = cv2.minAreaRect(con1)
                height1 = max(rect1[1])
                # Img Height and Img Width
                imgHieght, imgWidth, _ = img.shape
                # Contours Corners
                box1 = np.int0(cv2.boxPoints(rect1))
                # prints the contours
                cv2.drawContours(gray, [box1], 0, (255, 255, 255), 2)
                # finds both middle points for both contours
                midX, midY = calcMid(b=box1[1], d=box1[3])
                # calculate the angle
                angle = calcAngle(img=img, midX=midX, tanFOV=tanFov)
                distance = calcDistance(img, height1, 47, 5)
                #axis m1011 = 47
                if (abs(angleCounter1 - angle) < 2):
                    distanceCheck(img, height1, 47, 5, counter2)
                    printAngle(angle, counter2)
                    counter2 = 0
                elif (counter2 is 0):
                    if (abs(distance - prevDistance) > 5 and prevDistance != 0):
                        if (distance > prevDistance):
                            print "distance", distance
                            print "angle", angle
                            counter2 = 1
                        else:
                            print "distance,", prevDistance
                            print "angle", angle
                            counter2 = 1
                    else:
                        print "distance", distance
                        print "angle", angle
                        prevDistance = distance
                        counter2 = 1
                #prints the angle
                if (angleCounter1 is 0):
                    angleCounter1 = angle
                if (prevDistance is 0):
                    prevDistance = distance
                cv2.imshow("gray", gray)
                #returns angle to the robot
                table.putValue("angle", angle)
            counter3 = 0
        #returns nothing if cant find anything
        if(x is 0):
            table.putValue("angle",0)
            printNoRetroFound(x, counter3)
            counter3 = 1
        key = 0xff & cv2.waitKey(1)
        if key == 27:
            break
