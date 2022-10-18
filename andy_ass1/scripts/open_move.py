# open_move.py
# Moves until it sees green / brown grape vine
# avoids objects

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import json
import random
from math import sqrt

#########
# class #
#########
class OpenMove:

    ########
    # init #
    ########
    def __init__(self, distance=1, speed=1):
        self.cmdMap = {'fwd': 'front', 'rev': 'back', 'lft': 'left', 'rgt': 'right'}
        self.od = {}
        self.direction = 'fwd'
        self.distance = distance
        self.maxSpeed = speed
        
        self.publisher = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/object_distance", String, self.callback)
        rospy.Subscriber("/thorvald_001/robot_pose", Pose, self.callbackPose)
        
        # pid stuff
        # Taken from https://www.youtube.com/watch?v=gbMUOgJInYs
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.previous_error = 0
        self.previous_integral = 0
        self.target = 0

        # Are callbacks active? These both need to be true
        self.activeD = False
        self.activeP = False

        # Pose related
        self.positionX = 0.0
        self.positionY = 0.0
        self.positionZ = 0.0
        self.positionAtTurn = (0.0, 0.0)

        # distance related
        self.distanceTravelled = 0
        self.initialPosition = (0.0, 0.0)

    ##########
    # active #
    ##########
    def active(self):
        if self.activeD and self.activeP:
            return True
        return False

    #######
    # pid #
    #######
    # Taken from https://www.youtube.com/watch?v=gbMUOgJInY
    def pid(self, dir):

        pass

    #############
    # kmToMiles # 
    #############
    def kmToMiles(self, metres):
        return (metres / 1000) * 0.621371

    ################
    # callbackPose #
    ################
    def callbackPose(self, data):
        ''' Listens for pose data '''
        
        # NOTE x and Y are reversed - why - is would rotated?
        self.positionX = data.position.y
        self.positionY = data.position.x
        self.positionZ = data.position.z
        #print("X: ", round(self.positionX,2),"Y:", round(self.positionY,2))
        if not self.activeP:
            self.initialPosition = (self.positionX, self.positionY)
            self.positionAtTurn = (self.positionX, self.positionY)
        self.activeP = True

    ###################
    # getDistanceCrow #
    ###################
    def getDistanceCrow(self):
        # initial position to current position
        distance = 0
        distanceX = abs(self.positionX - self.initialPosition[0])
        distanceY = abs(self.positionY - self.initialPosition[1])
        distance = sqrt((distanceX * distanceX) + (distanceY * distanceY))
        return distance
        
    ####################    
    # getDistanceTotal #
    ####################
    def getDistanceTotal(self):
        # add distance since last turn to total
        distance = 0
        distanceX = abs(self.positionX - self.positionAtTurn[0])
        distanceY = abs(self.positionY - self.positionAtTurn[1])
        distance = sqrt((distanceX * distanceX) + (distanceY * distanceY))
        self.distanceTravelled += distance
        

    ############
    # callback #
    ############
    def callback(self, data):
        ''' Listens for object distances '''
        self.od = json.loads(data.data)
        self.activeD = True

    def pub(self, cmd):
        if not self.active(): return
        self.publisher.publish(cmd)
        
    def decision(self):
        if not self.active(): return

        if self.od[self.cmdMap[self.direction]] < self.distance :
            self.stop()
            self.chooseRndDir()
            self.pub(self.makeCmd())
        else:
            self.pub(self.makeCmd())
        

    def chooseRndDir(self):
        ''' picks a random direction that is not current direction '''
        self.getDistanceTotal()
        newDir = []
        for key in self.cmdMap.keys():
            if key != self.direction:
                newDir.append(key)
        self.direction = newDir[random.randint(0, len(newDir)-1)]

        # Mark position for distance calc
        self.positionAtTurn = (self.positionX, self.positionY)
        
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
        print("Distance for Crow: " + str(round(move.getDistanceCrow(),2)) + "m")
        print("Total Distance: " + str(round(move.distanceTravelled,2)) + "m - " + str(round(move.kmToMiles(move.distanceTravelled),2)) + "miles")
        print()
        rate.sleep()

if __name__ == '__main__':
    main()