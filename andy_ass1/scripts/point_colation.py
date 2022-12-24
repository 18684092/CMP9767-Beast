import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import  Point

class Collate:

    ########
    # init #
    ########
    def __init__(self):

        self.bunches = []
        self.pcBunches = PointCloud()

        left = rospy.Subscriber('/thorvald_001/grapes_left', PointCloud, self.left_callback)
        right = rospy.Subscriber('/thorvald_001/grapes_right', PointCloud, self.right_callback)
        front = rospy.Subscriber('/thorvald_001/grapes_front', PointCloud, self.front_callback)

        self.grapeBunches = rospy.Publisher('/thorvald_001/grape_bunches', PointCloud, queue_size=10)

    def left_callback(self, data):
        self.collate_points(data)

    def right_callback(self, data):
        self.collate_points(data)       

    def front_callback(self, data):
        self.collate_points(data)

    def publish(self):
        pcBunches = PointCloud()
        pcBunches.header.frame_id = "map"
        pcBunches.header.stamp = rospy.Time.now()
        for pointDict in self.bunches:
            if pointDict['n'] > 10:
                p = Point()
                p.x = pointDict['x']
                p.y = pointDict['y']
                p.z = pointDict['z']
                pcBunches.points.append(p)
        self.grapeBunches.publish(pcBunches)
        print(len(pcBunches.points))
        print()


    def collate_points(self, pc):
        for point in pc.points:
            # Is point within 5cm of stored 3D point?
            x = round(point.x,2)
            y = round(point.y,2)
            z = round(point.z,2) 
            found = False
            for i, pointDict in enumerate(self.bunches):
                sX = pointDict['x']
                sY = pointDict['y']
                sZ = pointDict['z']
                sN = pointDict['n']
                dX = abs(abs(sX) - abs(x))
                dY = abs(abs(sY) - abs(y))
                dZ = abs(abs(sZ) - abs(z))
                if dX < 0.5 and dY < 0.5 and dZ < 0.5:
                    self.bunches[i]['n'] += 1
                    found = True
            if not found:
                self.bunches.append({'x':x, 'y':y, 'z':z, 'n':1 })

        

########
# main #
########
def main():
    rospy.init_node('bunches', anonymous=True)
    collate = Collate()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():  
        collate.publish()   
        rate.sleep()

if __name__ == '__main__':
    main()