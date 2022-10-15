# open_move.py
# Moves until it sees green / brown grape vine
# avoids objects

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import random

class OpenMove:
    def __init__(self, distance=1, speed=1):
        self.cmdMap = {'fwd': 'front', 'rev': 'back', 'lft': 'left', 'rgt': 'right'}
        self.od = {}
        self.direction = 'fwd'
        self.distance = distance
        self.speed = speed
        self.state = 'go'
        self.active = False
        self.publisher = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/object_distance", String, self.callback)
        

    def callback(self, data):
        ''' listens for objects '''
        s = data.data
        self.od = json.loads(s)
        self.active = True

        

    def pub(self, cmd):
        if not self.active: return
        self.state = 'go'
        self.publisher.publish(cmd)
        

    def decision(self):
        if not self.active: return
        print(self.od[self.cmdMap[self.direction]])
        if self.od[self.cmdMap[self.direction]] < self.distance * 4:
            self.pub(self.makeCmd('slow'))
        if self.od[self.cmdMap[self.direction]] < self.distance :
            self.pub(self.makeCmd('stop'))
            self.chooseRndDir()
            self.pub(self.makeCmd('normal'))
        else:
            self.pub(self.makeCmd('normal'))
        

    def chooseRndDir(self):
        ''' picks a random direction that is not current direction '''
        newDir = []
        for key in self.cmdMap.keys():
            if key != self.direction:
                newDir.append(key)
        self.direction = newDir[random.randint(0, len(newDir)-1)]
        print(self.direction)
        

    def makeCmd(self, speed = 'normal'):
        x = 0.0
        y = 0.0
        z = 0.0
        fwdSpeed = self.speed
        if speed == 'slow':
            fwdSpeed /= 4
        t = Twist()
        if speed == 'stop':
            self.state = 'stop'
        if self.state == 'stop':
            x, y, z = 0.0, 0.0, 0.0
        elif self.direction == 'fwd':
            x = fwdSpeed
        elif self.direction == 'rev':
            x = -fwdSpeed
        elif self.direction == 'lft':
            y = fwdSpeed
        elif self.direction == 'rgt':
            y = -fwdSpeed
        t.linear.x = x
        t.linear.y = y
        t.angular.z = z
        return t

    def stop(self):
        self.pub(self.makeCmd('stop'))


def main():
    rospy.init_node('open_mover')
    move = OpenMove(1.5,1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        move.decision()
        rate.sleep()

if __name__ == '__main__':
    main()