#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import json

#

class Distance:
    """
    A very simple Roamer implementation for Thorvald.
    """

    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to laser scans and publish the distance of an
        object around the robot
        """

        self.distance_back = {}
        self.distance_front = {}
        self.distance = {}
        self.activeB = False
        self.activeF = False
        self.publisher = rospy.Publisher('/thorvald_001/object_distance', String, queue_size=1)
        rospy.Subscriber("/thorvald_001/back_scan", LaserScan, self.callback_back)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback_front)

    def callback_back(self, data):
        """
        Callback called any time a new laser scan becomes available
        """
        self.activeB = True
        self.distance_back['back'] = min(data.ranges[450:600])
        self.distance_back['left']= min(data.ranges[100:250])
        self.distance_back['right'] = min(data.ranges[710:719]) -1.25
        self.distance_back['front'] = min(data.ranges[0:3]) - 1.5

    def callback_front(self, data):
        """
        Callback called any time a new laser scan becomes available
        """
        self.activeF = True
        self.distance_front['back'] = min(data.ranges[0:2]) - 1.5
        self.distance_front['left']= min(data.ranges[710:719]) - 1.25
        self.distance_front['right'] = min(data.ranges[220:250])
        self.distance_front['front'] = min(data.ranges[450:500])

    def pub(self):
        if self.activeF and self.activeB:
            self.distance['back'] = round(min(self.distance_back['back'], self.distance_front['back']),2)
            self.distance['left'] = round(min(self.distance_back['left'], self.distance_front['left']),2)
            self.distance['right'] = round(min(self.distance_back['right'], self.distance_front['right']),2)
            self.distance['front'] = round(min(self.distance_back['front'], self.distance_front['front']),2)
            d_str = json.dumps(self.distance)
            self.publisher.publish(d_str)

def main():
    rospy.init_node('object_distance')
    distanceToObject = Distance()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        distanceToObject.pub()
        rate.sleep()

if __name__ == '__main__':
    main()
        

