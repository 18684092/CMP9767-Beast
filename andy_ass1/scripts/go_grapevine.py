###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

import rospy
import actionlib
from std_msgs.msg import String

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

# Waypoints - first few are for localisation, then travel along grapevine, then return home
nodeList = [1, 3, 4, 105,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 1, 105]

# After thought - TODO combine with nodeList - we need to know which way point is for which side of the grapevine
row = {10:1, 11:1, 12:1, 13:1, 14:1, 15:1, 16:1, 17:1, 18:1, 21:2, 22:2, 23:2, 24:2, 25:2, 26:2, 27:2, 28:2, 29:2}

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

        # No point continuing until the action client is ready
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()

        # Publisher notifies camera when to operate
        self.move = rospy.Publisher('/thorvald_001/moving', String, queue_size=1, latch=True)
        self.state = rospy.Publisher('/thorvald_001/state', String, queue_size=1, latch=True)
        self.row = rospy.Publisher('/thorvald_001/row', String, queue_size=10, latch=True)
        
        # This node waits for the camera to be ready before moving to next node
        rospy.Subscriber("/thorvald_001/camera_done", String, self.callback)

        # Cycle through all nodes
        self.goGrapes()

    ############
    # goGrapes #
    ############
    def goGrapes(self):

        # Move through each waypoint
        for i, wp in enumerate(nodeList):
            # Program state
            if i >=0 and i < 5:
                self.state.publish(String('Localising position'))
            if i >=5 and i <26:
                self.state.publish(String('Counting bunches of grapes'))
            if i >=26:
                self.state.publish(String('Returning home'))

            # Notify camera node that robot is moving
            self.move.publish(String('true'))

            # Goto goal
            goal = GotoNodeGoal()
            goal.target = "waypoint" + str(wp)
            self.client.send_goal(goal)
            status = self.client.wait_for_result()
            result = self.client.get_result()

            # If we are at a grapevine node and normal operation
            if result.success == True and ((wp >= 10 and wp <= 18) or (wp >= 21 and wp <= 29)):
                self.move.publish(String('false'))
                
                # Publish the row number
                self.row.publish((String("row" + str(row[wp]))))
                
                # wait for camera node to finish
                self.cameraResult = "imaging"
                while self.cameraResult == "imaging":
                    pass
        
        # The robot has returned home
        self.state.publish(String('Finished'))


    ############
    # callback #
    ############
    def callback(self, data):
        self.cameraResult = data


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