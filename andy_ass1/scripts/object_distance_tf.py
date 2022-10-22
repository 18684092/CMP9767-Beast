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
        
        self.listener = tf.TransformListener()
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


    ##################
    # callback_front #
    ##################
    def callback_front(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        self.activeF = True

        min_dist = min(data.ranges)
        laser_point_2d = [] * len(data.ranges)
        for index,value in enumerate(data.ranges):
            # NOTE x and y have been rotated
            x = sin(ang) * value
            y = cos(ang) * value

            angle = data.angle_min + (index * data.angle_increment)

            laser_point_2d[index] = [cos(angle) * value, sin(angle) * value, 0.0, angle]
        
            print()


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
        
