#!/usr/bin/env python

import rospy
import cv2
import numpy as np


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

        # Blank screen image
        self.img = np.zeros((512,512,3), np.uint8)
        
        
        # Font stuff
        self.font                   = cv2.FONT_HERSHEY_SIMPLEX
        self.bottomLeft = (10,500)
        self.fontScale              = 1
        self.fontColor              = (255,255,255)  
        self.thickness              = 1
        self.lineType               = 2

        cv2.namedWindow("AndyAss1")

    ##############
    # drawScreen #
    ##############
    def drawScreen(self):

        cv2.imshow("AndyAss1",self.img)
        

    def addText(self):
        cv2.putText(self.img,
            'Hello World!',
            self.bottomLeft,
            self.font,
            self.fontScale,
            self.fontColor,
            self.thickness,
            self.lineType)







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