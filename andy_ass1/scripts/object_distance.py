###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import json
from math import sqrt, sin, cos, degrees

############
# Distance #
############
class Distance:
    """
    A very simple object detection implementation for Thorvald.
    """

    ########
    # init #
    ########
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to laser scans and publish the distance of an
        object around the robot
        """

        self.distance_back = {'back':999, 'front':999, 'left':999, 'right':999, 'nearest':999}
        self.distance_front = {'back':999, 'front':999, 'left':999, 'right':999, 'nearest': 999}
        self.distance = {'back':999, 'front':999, 'left':999, 'right':999, 'nearest': 999}

        # We haven't received a reading yet - True when we have a laser scan
        self.activeB = False
        self.activeF = False
        
        self.publisher = rospy.Publisher('/thorvald_001/object_distance', String, queue_size=1)
        rospy.Subscriber("/thorvald_001/back_scan", LaserScan, self.callback_back)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback_front)

    #################
    # callback_back #
    #################
    def callback_back(self, data):
        """
        Callback called any time a new laser scan becomes available
        """
        self.activeB = True
        # Forty-Five degrees rotated
        # 0   Degrees = North
        # 90  degrees = East
        # 180 degrees = South
        ff = 0.785398
        inc = 0.006314325612038374
        ang = 2.359999895095825 + ff
        f = [999]
        b = [999]
        l = [999]
        r = [999] 

        # calculate all x,y distances
        for index,value in enumerate(data.ranges):
            # NOTE x and y have been rotated
            x = sin(ang) * value
            y = cos(ang) * value

            angle = degrees(ang)
            ang -= inc
            if angle > -90 and angle < 90 and x > -1.45 and x < 0.4 and y > 0:
                b.append(abs(y))
            if angle < -79:
                r.append(abs(value) - 1.25)
            if y < 0 and y > -1.70  and x > 0:
                l.append(abs(x))
            if angle > 179:
                f.append(abs(value) - 1.75)
            if angle < 0.1 and angle > 0.1:
                b.append(abs(y))
                l.append(abs(x))
           # print(index, angle, x, y, value)
        
        self.distance_back['front'] = min(f)
        self.distance_back['right'] = min(r)
        self.distance_back['back'] = min(b)
        self.distance_back['left']= min(l)

    ##################
    # callback_front #
    ##################
    def callback_front(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        self.activeF = True
   
        # Forty-Five degrees rotated
        # 0   Degrees = North
        # 90  degrees = East
        # 180 degrees = South
        ff = 0.785398
        inc = 0.006314325612038374
        ang = 2.359999895095825 + ff
        f = [999]
        b = [999]
        l = [999]
        r = [999] 
 
        for index,value in enumerate(data.ranges):
            # NOTE x and y have been rotated
            x = sin(ang) * value
            y = cos(ang) * value

            angle = degrees(ang)
            ang -= inc
            if angle > -90 and angle < 90 and x > -1.45 and x < 0.4 and y > 0:
                f.append(abs(y))
            if angle < -79:
                l.append(abs(value) - 1.25)
            if y < 0 and y > -1.70  and x > 0:
                r.append(abs(x))
            if angle > 179:
                b.append(abs(value) - 1.75)
            if angle < 0.1 and angle > 0.1:
                f.append(abs(y))
                r.append(abs(x))
        
        self.distance_front['front'] = min(f)
        self.distance_front['right'] = min(r)
        self.distance_front['back'] = min(b)
        self.distance_front['left']= min(l) 

    #######
    # pub #
    #######
    def pub(self):
        if self.activeF and self.activeB:
            self.distance['back'] = min(self.distance_front['back'], self.distance_back['back'])
            self.distance['left'] = min(self.distance_front['left'], self.distance_back['left'])
            self.distance['right'] = min(self.distance_front['right'], self.distance_back['right'])
            self.distance['front'] = min(self.distance_front['front'], self.distance_back['front'])
            self.distance['nearest'] = min(self.distance['front'], min(self.distance['back'], min(self.distance['right'],self.distance['left'])))
            d_str = json.dumps(self.distance)
            self.publisher.publish(d_str)

########
# main #
########
def main():
    rospy.init_node('object_distance')
    distanceToObject = Distance()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        distanceToObject.pub()
        rate.sleep()

##################
# start properly #
##################
if __name__ == '__main__':
    main()
        
