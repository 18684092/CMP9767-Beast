# open_move.py
# Moves until it sees green / brown grape vine
# avoids objects

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
import json
import random
from math import sqrt

# TODO added namespace parameter and other params
# TODO get PID control working

#########
# class #
#########
class OpenMove:

    ########
    # init #
    ########
    def __init__(self, objectDistance=1, speed=0.3):
        self.cmdMap = {'fwd': 'front', 'rev': 'back', 'lft': 'left', 'rgt': 'right'}
        self.od = {}
        self.direction = 'fwd'
        self.distance = objectDistance
        self.maxSpeed = speed



        # Publish topics
        self.publisher = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
        self.publisherDistance = rospy.Publisher('/thorvald_001/distance_travelled', String, queue_size=1)
        
        # Subscriber topics
        rospy.Subscriber("/thorvald_001/object_distance", String, self.callback)
        rospy.Subscriber("/thorvald_001/robot_pose", Pose, self.callbackPose)
        
        # pid stuff
        # Taken from https://www.youtube.com/watch?v=gbMUOgJInYs
        self.Kp = 0.01
        self.Ki = 0.00001
        self.Kd = 0.000001
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
    def pid2(self):
        t = 0.0
        d = 1
        if self.direction == 'fwd' or self.direction == 'rev':
            t = self.positionX
        else:
            t = self.positionY
        error = self.target - t
        integral = self.previous_integral + error
        if error > self.previous_error:
            d = -1
        else: 
            d = 1
        derivative = error - self.previous_error
        z = abs((self.Kp * error) + (self.Ki * integral) + (self.Kd * derivative))
        rospy.loginfo("Perror : %s" % self.previous_error)
        self.previous_error = error
        self.previous_integral = integral
        z *= d
        rospy.loginfo("z      : %s" % str(z))
        rospy.loginfo("error  : %s" % error)
        
        rospy.loginfo("current: %s" % t)
        rospy.loginfo("Target : %s" % self.target)
        rospy.loginfo("")

        
        return z

    def pid(self):
        t = 0
        z = 0
        d = 0.02
        if self.direction == 'fwd' or self.direction == 'rev':
            t = (self.positionX)
        else:
            t = (self.positionY)

        error = self.target - t
        if self.direction == "fwd" or self.direction == "rgt":
            if error > self.previous_error:
                z = d
            else:
                z = -d
        if self.direction == "rev" or self.direction == "lft":
            if error > self.previous_error:
                z = -d
            else:
                z = d
        if abs(error) < 0.1:
            z = 0
        self.previous_error = error
        return z


    ################
    # setPIDTarget #
    ################
    def setPIDTarget(self):
        if self.direction == 'fwd' or self.direction == 'rev':
            self.target = self.positionX
        else:
            self.target = self.positionY

        self.previous_integral = 0.0
        self.previous_error = 0.0
        self.positionZ = 0.0

    ###############
    # pubDistance #
    ###############
    def pubDistance(self):
        distance = {}
        distance['crow'] = {"meters": round(self.getDistanceCrow(),2)}
        distance['total'] = {"miles": round(self.kmToMiles(self.distanceTravelled),2)}
        self.publisherDistance.publish(String(json.dumps(distance)))

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

        # NOTE x and Y are reversed - why - is world rotated?
        self.positionX = data.position.y
        self.positionY = data.position.x
        self.positionZ = data.position.z
        
        if not self.activeP:
            self.initialPosition = (self.positionX, self.positionY)
            self.positionAtTurn = (self.positionX, self.positionY)
            self.setPIDTarget()
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

    #######
    # pub #
    #######
    def pub(self, cmd):
        if not self.active(): return
        self.publisher.publish(cmd)
        
    ############
    # decision #
    ############
    def decision(self):
        if not self.active(): return
        if self.od[self.cmdMap[self.direction]] < self.distance :
            self.stop()
            self.chooseRndDir()
        self.pub(self.makeCmd())

        
    ################
    # chooseRndDir #
    ################
    def chooseRndDir(self):
        ''' picks a random direction that is not current direction '''

        # Calc distance before we move
        self.getDistanceTotal()
        
        newDir = []
        for key in self.cmdMap.keys():
            if key != self.direction:
                newDir.append(key)
        self.direction = newDir[random.randint(0, len(newDir)-1)]

        # Mark position for distance calc
        self.positionAtTurn = (self.positionX, self.positionY)
        self.setPIDTarget()

    ###########
    # makeCmd #
    ###########
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

        # turn with pid value
        t.angular.z = self.pid()

        return t

    ########
    # stop #
    ########
    def stop(self):
        self.pub(self.makeCmd('stop'))

########
# main #
########
def main():
    rospy.init_node('open_mover')
    try:
        speed = float(rospy.get_param('~speed'))
    except:
        speed = 0.3 
    move = OpenMove(1,speed)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        move.decision()
        move.pubDistance()
        
        rate.sleep()

if __name__ == '__main__':
    main()