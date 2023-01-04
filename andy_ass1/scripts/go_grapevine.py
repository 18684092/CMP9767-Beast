###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

import rospy
import actionlib

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

# Waypoints
nodeList = [1, 3, 4, 105,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 1, 105]

# The first few node traversals are to allow AMCL to localise, after that goto the grapevine and
# pause for each image

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client', anonymous=True)
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()

    for wp in nodeList:
        goal = GotoNodeGoal()
        goal.target = "waypoint" + str(wp)
        client.send_goal(goal)
        status = client.wait_for_result()
        result = client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)

