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
from sensor_msgs.msg import PointCloud
from math import sqrt
import json
import time

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

        self.start = time.time()
        self.end = time.time()

        # Subscribers relating to robot and object positions
        rospy.Subscriber("/thorvald_001/object_distance", String, self.callbackDistance)
        rospy.Subscriber("/thorvald_001/robot_pose", Pose, self.callbackPose)
        
        # Subscribers for what the robot is doing
        rospy.Subscriber("/thorvald_001/topological_navigation/goal", GotoNodeActionGoal, self.callbackGoal,queue_size=10)
        rospy.Subscriber("/thorvald_001/moving", String, self.callbackMoving)
        rospy.Subscriber("/thorvald_001/camera_done", String, self.callbackCamera)
        rospy.Subscriber("/thorvald_001/state", String, self.callbackState)

        # Subscribers for data
        rospy.Subscriber('/thorvald_001/grape_bunches', PointCloud, self.grapes_callback)
        rospy.Subscriber('/thorvald_001/row_widths', String, self.callbackWidths)

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
        
        # Object Distance dictionary
        self.od = {}

        # States and data
        self.moving = "false"
        self.camera = "not imaging"
        self.state = 'Initialising'

        self.numberBunches = 0
        self.numberGrapes = 0
        self.widths = {}

    ####################
    # Object detection #
    ####################
    def callbackWidths(self, data):
        self.widths = json.loads(data.data)
        print(self.widths)

    #########
    # State #
    #########
    def callbackState(self, data):
        self.state = data.data

    ###########
    # Bunches #
    ###########
    def grapes_callback(self, pc):
        self.numberBunches = len(pc.points)

    ##########
    # Moving #
    ##########
    def callbackMoving(self, data):
        self.moving = data.data

    ###############
    # Camera busy #
    ###############
    def callbackCamera(self, data):
        self.camera = data.data

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
        
    ###########
    # addText #
    ###########
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
        moving = "Robot moving: " + str(self.moving)
        camera = "Camera state: " + str(self.camera)
        bunches = "Bunches of grapes: " + str(self.numberBunches)
        state = "Control state: " + str(self.state)
        if str(self.state) != "Finished":
            self.end = time.time()

        # time formatting taken from https://stackoverflow.com/questions/27779677/how-to-format-elapsed-time-from-seconds-to-hours-minutes-seconds-and-milliseco
        hours, rem = divmod(self.end - self.start, 3600)
        minutes, seconds = divmod(rem, 60)
        timing = "Time taken: {:0>2} mins {:05.2f} seconds".format(int(minutes),seconds)
        
        try:
            widths1 = min(float(self.widths['row1']['min']) ,  float(self.widths['row2']['min']))
            widths2 = max(float(self.widths['row1']['max']) ,  float(self.widths['row2']['max']))
            if widths1 < 0 and widths2 < 0:
                w = abs(widths1) - abs(widths2)
            elif  widths1 < 0 and widths2 >= 0:
                w = abs(widths1) + abs(widths2)
            gWidth1 = "Grapevine length: " + str(round(w , 2)) + " m"
 
            bunchMetre = "Average bunches per metre: " + str(round(self.numberBunches / (w)))
        except:
            gWidth1 = "Grapevine length: 0 m"
            avgWidth = 0
            bunchMetre = "Average bunches per metre: 0"
        cv2.putText(self.img, state, (10,15), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, distance, (10,45), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, distanceCrow, (10,60), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, oleft, (10,75), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, oright, (10,90), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, ofront, (10,105), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, oback, (10,120), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, rpos, (10,135), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)

        cv2.putText(self.img, target, (10,170), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, moving, (10,185), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, camera, (10,200), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)

        cv2.putText(self.img, bunches, (10,235), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, gWidth1, (10,250), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)

        cv2.putText(self.img, bunchMetre, (10,280), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)
        cv2.putText(self.img, timing, (10,315), self.font, self.fontScale, self.fontColor, self.thickness, self.lineType)


    #############
    # kmToMiles # 
    #############
    def kmToMiles(self, metres):
        return (metres / 1000) * 0.621371

    ####################
    # Topological goal #
    ####################
    def callbackGoal(self, data):
        self.target = data.goal.target

    ################
    # callbackPose #
    ################
    def callbackPose(self, data):
        ''' Listens for pose data '''
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