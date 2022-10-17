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
        self.maxSpeed = speed
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
        self.publisher.publish(cmd)
        

    def decision(self):
        if not self.active: return

        if self.od[self.cmdMap[self.direction]] < self.distance :
            self.stop()
            self.chooseRndDir()
            self.pub(self.makeCmd())
        else:
            self.pub(self.makeCmd())
        

    def chooseRndDir(self):
        ''' picks a random direction that is not current direction '''
        newDir = []
        for key in self.cmdMap.keys():
            if key != self.direction:
                newDir.append(key)
        self.direction = newDir[random.randint(0, len(newDir)-1)]
        
    def makeCmd(self, speed = 'normal'):
        x, y, z = 0.0, 0.0, 0.0
        t = Twist()
        fwdSpeed = self.maxSpeed

        if self.od['nearest'] < self.distance :
            fwdSpeed = self.od['nearest'] / self.distance
            if fwdSpeed > self.maxSpeed:
                fwdSpeed = self.maxSpeed
            if fwdSpeed < 0.3:
                fwdSpeed = 0.3

        
        if speed == 'stop':
            self.state = 'stop'
            x, y, z = 0.0, 0.0, 0.0
            return t
        
        if self.direction == 'fwd':
            t.linear.x = fwdSpeed
        elif self.direction == 'rev':
            t.linear.x = -fwdSpeed
        elif self.direction == 'lft':
            t.linear.y = fwdSpeed
        elif self.direction == 'rgt':
            t.linear.y = -fwdSpeed

        return t

    def stop(self):
        self.pub(self.makeCmd('stop'))

def main():
    rospy.init_node('open_mover')
    move = OpenMove(1,1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        move.decision()
        rate.sleep()

if __name__ == '__main__':
    main()