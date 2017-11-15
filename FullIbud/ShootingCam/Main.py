import sys
sys.path.append("/home/pi/opencv-3.1.0/build/lib")
import cv2
import GearCam
import ShootingCam
from networktables import NetworkTables
import numpy as np
import math
import threading


#Connects to the camera, sets the angle to 0
def createNetworkTables():
    NetworkTables.setIPAddress("10.45.86.2") #Sets the IP address
    NetworkTables.setClientMode() #Sets the client mode to the table
    NetworkTables.initialize() #Intializes the table
    table = NetworkTables.getTable('imgProc') #Gets the variable table from the computer
    table.putValue("angleGear", 0) #Sets the gear angle to 0
    table.putValue("angleShoot", 0) #Sets the shooting angle to 0
    table.putValue("distanceGear", 0) #Sets the distance from the gear to 0
    table.putValue("distanceShoot", 0) #Sets the distance from the boiler to 0
    table.putValue("camera", 1) #gets what camera you want to process
    return table #returns the table


if __name__ == "__main__": #main
    shootingProc = ShootingCam.ShootingCam() #gets an object from the shootingCam class
    gearProc = GearCam.Gear() #gets an object from the gearCam class
    table = createNetworkTables() #Gets the table from the operation above
    print "done" #Prints done when he enters the while
    while (True):
        camera = int(table.getNumber("camera")) #Gets the camera number to use
        contourShooting, ImgShooting = shootingProc.getImage() #Gets the contour and the Img from the shooting camera
        contourGear, ImgGear = gearProc.getImage() #Gets the contour and the Img from the gear camera
        cv2.imshow("shooting", ImgShooting) #Prints the shooting img
        cv2.imshow("shootingMask", shootingProc.mask) #Prints the shooting mask
        cv2.imshow("gear", ImgGear) #Prints the gear Img
        cv2.imshow("gearMask", gearProc.mask) #Prints the gear Mask
        if (camera is 1): #Checks what camera he should use
            shootingProc.shootCam(table) #run the entire proccessing operation for the shooting cmaera
        elif (camera is 2): #checks if the camera is the gear one
            gearProc.gearCam(table) #runs the entire proccessing operation for the gear
        else:
            print "No Camera Chosen" #In a no camera case
        key = 0xff & cv2.waitKey(1) #gets the key that is pressed
        if key == 110: #stops when escape is clicked
            break #breaks the loop