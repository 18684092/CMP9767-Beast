###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

import rospy
import actionlib
from std_msgs.msg import String

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

# Waypoints
nodeList = [1, 3, 4, 105,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 1, 105]

# The first few node traversals are to allow AMCL to localise, after that goto the grapevine and
# pause for each image

########
# Move #
########
class Move():

    ############
    # __init__ #
    ############
    def __init__(self, nodes):

        self.cameraResult = 'imaging'

        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
        self.move = rospy.Publisher('/thorvald_001/moving', String, queue_size=1, latch=True)
        rospy.Subscriber("/thorvald_001/camera_done", String, self.callback)

        self.goGrapes()

    ############
    # goGrapes #
    ############
    def goGrapes(self):
        for wp in nodeList:
            self.move.publish(String('true'))
            goal = GotoNodeGoal()
            goal.target = "waypoint" + str(wp)
            self.client.send_goal(goal)
            status = self.client.wait_for_result()
            result = self.client.get_result()

            # If we are at a grapevine node and normal operation
            if result.success == True and ((wp >= 10 and wp <= 18) or (wp >= 21 and wp <= 29)):
                self.move.publish(String('false'))
                # wait for camera node to finish
                self.cameraResult = "imaging"
                while self.cameraResult == "imaging":
                    pass


    ############
    # callback #
    ############
    def callback(self, data):
        self.cameraResult = data
        print(self.cameraResult)

########
# main #
########
def main():
    rospy.init_node('topological_navigation_client', anonymous=True)

    move = Move(nodeList)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():     
        rate.sleep()

# Start properly
if __name__ == '__main__':
    main()