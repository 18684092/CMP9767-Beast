###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from topological_navigation_msgs.msg import GotoNodeActionGoal
from math import sqrt
import json

###########
# Display #
###########
class Display:
    '''
    Display a main information screen using OpenCV
    that reports robot activity.
    '''

    ########
    # init #
    ########
    def __init__(self):

        # Setup all subscribers
        # Subscriber topics
        rospy.Subscriber("/thorvald_001/object_distance", String, self.callbackDistance)
        rospy.Subscriber("/thorvald_001/robot_pose", Pose, self.callbackPose)
        
        rospy.Subscriber("/thorvald_001/topological_navigation/goal", GotoNodeActionGoal, self.callbackGoal,queue_size=10)

        # Blank screen image
        self.img = np.zeros((512,512,3), np.uint8)
        
        # Font stuff
        self.font                   = cv2.FONT_HERSHEY_SIMPLEX
        self.bottomLeft = (10,500)
        self.fontScale              = 0.5
        self.fontColor              = (255,255,255)  
        self.thickness              = 1
        self.lineType               = cv2.LINE_AA

        # Are callbacks active? These both need to be true
        self.activeD = False
        self.activeP = False

        # Pose related
        self.positionX = 0.0
        self.positionY = 0.0
        self.oldPositionX = 0.0
        self.oldPositionY = 0.0
        self.orientationZ = 0.0
        self.target = None

        # distance related
        self.distanceTravelled = 0
        self.initialPosition = (0.0, 0.0)
        self.od = {}

    ##########
    # active #
    ##########
    def active(self):
        if self.activeD and self.activeP:
            return True
        return False

        cv2.namedWindow("AndyAss1")

    ##############
    # drawScreen #
    ##############
    def drawScreen(self):
        cv2.imshow("AndyAss1",self.img)
        

    def addText(self):
        self.img = np.zeros((512,512,3), np.uint8)
        distance =     "Distance travelled: " + str(round(self.distanceTravelled,2)) + " m"
        distanceCrow = "Distance from start: " + str(round(self.getDistanceCrow(),2)) + " m"
        try:
            oleft = "Closest object on left: " + str(round(float(self.od['left']),2)) + " m"
            oright = "Closest object on right: " + str(round(float(self.od['right']),2)) + " m"
            ofront = "Closest object on front: " + str(round(float(self.od['front']),2)) + " m"
            oback = "Closest object on back: " + str(round(float(self.od['back']),2)) + " m"
        except:
            oleft = "Closest object: No data"
            oright = oleft
            oback = oleft
            ofront = oleft
        rpos = "Robot pose: x: " + str(round(self.positionX,2)) + " y: " + str(round(self.positionY,2)) + " rot: " + str(round(self.orientationZ,2)) 
        target = "Topological goal: " + str(self.target)
        cv2.putText(self.img, distance, (10,35), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, distanceCrow, (10,50), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, oleft, (10,65), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, oright, (10,80), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, ofront, (10,95), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, oback, (10,110), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, rpos, (10,125), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, target, (10,140), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)

    #############
    # kmToMiles # 
    #############
    def kmToMiles(self, metres):
        return (metres / 1000) * 0.621371

    def callbackGoal(self, data):
        self.target = data.goal.target

        


    ################
    # callbackPose #
    ################
    def callbackPose(self, data):
        ''' Listens for pose data '''
        # NOTE x and Y are reversed - why - is world rotated?
        self.oldPositionX = self.positionX
        self.oldPositionY = self.positionY
        self.positionX = data.position.x
        self.positionY = data.position.y
        self.orientationZ = data.orientation.z
        
        if not self.activeP:
            self.initialPosition = (self.positionX, self.positionY)
            self.oldPositionX = self.positionX
            self.oldPositionY = self.positionY
        self.activeP = True
        self.getDistanceTotal()

    ###################
    # getDistanceCrow #
    ###################
    def getDistanceCrow(self):
        # initial position to current position
        distance = 0
        distanceX = abs(self.positionX - self.initialPosition[0])
        distanceY = abs(self.positionY - self.initialPosition[1])
        distance = sqrt((distanceX * distanceX) + (distanceY * distanceY))
        return distance
        
    ####################    
    # getDistanceTotal #
    ####################
    def getDistanceTotal(self):
        # add distance since last update to total
        distance = 0
        distanceX = abs(self.positionX - self.oldPositionX)
        distanceY = abs(self.positionY - self.oldPositionY)
        distance = sqrt((distanceX * distanceX) + (distanceY * distanceY))
        self.distanceTravelled += distance
        
    ############
    # callback #
    ############
    def callbackDistance(self, data):
        ''' Listens for object distances '''
        self.od = json.loads(data.data)
        self.activeD = True





########
# main #
########
def main():

    rospy.init_node('main_display')
    display = Display()
    while not rospy.is_shutdown():
        display.addText()
        display.drawScreen()
        k = cv2.waitKey(100)
        if k == 27:
            cv2.destroyAllWindows()
            break        
        if cv2.getWindowProperty('AndyAss1',cv2.WND_PROP_VISIBLE) != -1:
       
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()