import rospy
import actionlib

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

# Waypoints
nodeList = [1, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 1]


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

