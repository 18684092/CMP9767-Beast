#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf import TransformListener
import json
from math import sqrt, sin, cos, degrees

# TODO add namespace as a param using rospy.get_param

############
# Distance #
############
class Distance:
    """
    An object detection publisher for Thorvald.
    """

    ########
    # init #
    ########
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to laser scans and publish the distance of an
        object closest to the robot
        """
        # We haven't received a reading yet - True when we have a laser scan
        self.activeB = False
        self.activeF = False

        # There are two LiDARs
        self.poseF = PoseStamped()
        self.poseB = PoseStamped()
        
        self.listener = TransformListener()
        
        self.posePub = rospy.Publisher('nearest_obstacle_pose', PoseStamped,queue_size=1)
        rospy.Subscriber("back_scan", LaserScan, self.callback_back)
        rospy.Subscriber("front_scan", LaserScan, self.callback_front)

    #################
    # callback_back #
    #################
    def callback_back(self, data):
        """
        Callback called any time a new laser scan becomes available
        """
        
        self.activeB = True
        (x, y, z), angle = self.getNearestCartesian(data)
        self.poseB = self.makePose(x,y,z,angle,data)

    ##################
    # callback_front #
    ##################
    def callback_front(self, data):
        '''
        Callback called any time a new laser scan becomes available
        '''
        
        self.activeF = True
        (x, y, z), angle = self.getNearestCartesian(data)
        self.poseF = self.makePose(x,y,z,angle,data)        

    ############
    # makePose #
    ############
    def makePose(self, x, y, z, angle, data):
        pose = PoseStamped()
        pose.header = data.header
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(0, 0, sin(angle/2), cos(angle/2))
        
        return pose

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
        '''
        Publishes the pose of the closest object from the one of the
        LiDARs for RVIZ use. But only when both LiDARS have scanned.
        '''
        
        if not self.activeB or not self.activeF:
            return

        # Which LiDAR sees closest object
        f = self.poseF.pose.position
        b = self.poseB.pose.position
        fd = (f.x * f.x) + (f.y * f.y)
        bd = (b.x * b.x) + (b.y * b.y)
        if fd <= bd:
            pose = self.poseF
        else:
            pose = self.poseB

        transformed_pose = self.listener.transformPose("thorvald_001/base_link", pose)
        self.posePub.publish(transformed_pose)

########
# main #
########
def main():
    
    rospy.init_node('object_distance_tf')
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
        
