# open_move.py
# Moves until it sees green / brown grape vine
# avoids objects

import rospy
from std_msgs.msg import String
from geometry_msgs import Twist
import json

class OpenMove:
    def __init__(self, distance=1, speed=1):

        pass

    def callback(self, data):

        pass

    def move(self, direction, speed):

        pass

    def pub(self):

        pass

    def decision(self)

def main():
    rospy.init_node('open_mover')
    move = OpenMove(1,1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        move.pub()
        rate.sleep()

if __name__ == '__main__':
    main()