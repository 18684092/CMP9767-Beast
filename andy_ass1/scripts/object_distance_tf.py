#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf import TransformListener
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

        self.poseF = PoseStamped()
        self.poseB = PoseStamped()
        
        self.listener = TransformListener()
        self.publisher = rospy.Publisher('/thorvald_001/object_distance', String, queue_size=1)
        self.pose_pub = rospy.Publisher(
            '/nearest_obstacle',
            PoseStamped,queue_size=1
        )
        rospy.Subscriber("/thorvald_001/back_scan", LaserScan, self.callback_back)
        rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.callback_front)

    #################
    # callback_back #
    #################
    def callback_back(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        # Used to control when to publish
        self.activeB = True

        (x, y, z), angle = self.getNearestCartesian(data)
        
        self.poseB.header = data.header
        self.poseB.pose.position = Point(x, y, z)
        self.poseB.pose.orientation = Quaternion(0, 0, sin(angle/2), cos(angle/2))

    ##################
    # callback_front #
    ##################
    def callback_front(self, data):
        '''
        Callback called any time a new laser scan becomes available
        '''

        # Used to control when to publish
        self.activeF = True

        (x, y, z), angle = self.getNearestCartesian(data)
        
        self.poseF.header = data.header
        self.poseF.pose.position = Point(x, y, z)
        self.poseF.pose.orientation = Quaternion(0, 0, sin(angle/2), cos(angle/2))

    #######################
    # getNearestCartesian #
    #######################
    def getNearestCartesian(self, data):
        min_distance = data.range_max
        angle = data.angle_max
        for index,value in enumerate(data.ranges):
            if value < min_distance:
                min_distance = value
                angle = data.angle_min + (index * data.angle_increment)

        return (self.cartesian(angle, min_distance)), angle


    #############
    # cartesian #
    #############
    # Taken from https://www.mathsisfun.com/polar-cartesian-coordinates.html       
    def cartesian(self, angle, distance):
        return (distance * cos(angle), distance * sin(angle), 0.0)

    #######
    # pub #
    #######
    def pub(self):
        if not self.activeB or not self.activeF:
            return
        f = self.poseF.pose.position
        b = self.poseB.pose.position
        fd = sqrt((f.x * f.x) + (f.y * f.y))
        bd = sqrt((b.x * b.x) + (b.y * b.y))

        if fd <= bd:
            pose = self.poseF
        else:
            pose = self.poseB



        transformed_pose = self.listener.transformPose("thorvald_001/base_link", pose)
        rospy.loginfo("The closest point in laser coords is at:\n%s" % pose)
        rospy.loginfo("The closest point in robot coords is at:\n%s" % transformed_pose)
        self.pose_pub.publish(pose)

########
# main #
########
def main():
    rospy.init_node('object_distance')
    distanceToObject = Distance()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        distanceToObject.pub()
        rate.sleep()

##################
# start properly #
##################
if __name__ == '__main__':
    main()
        
