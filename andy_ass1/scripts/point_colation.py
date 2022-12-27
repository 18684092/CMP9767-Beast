import rospy

from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32, PointField
from geometry_msgs.msg import  Point, Point32
from operator import itemgetter

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

        self.grapeBunches = rospy.Publisher('/thorvald_001/grape_bunches', PointCloud, queue_size=30, latch=False)

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
        intensities = []
        for i, pointDict in enumerate(self.bunches):
            if pointDict['n'] > 5:
                p = Point32()
                intensities.append(pointDict['i'])
                p.x = pointDict['x']
                p.y = pointDict['y']
                p.z = pointDict['z']
                pcBunches.points.append(p)
        c = ChannelFloat32()
        c.name = "intensity"
        c.values = intensities
        pcBunches.channels=[c]       
        self.grapeBunches.publish(pcBunches)
        print(len(pcBunches.points))
       

    def collate_points(self, pc):
        for point,channel in zip(pc.points, pc.channels):
            # Is point within xcm of stored 3D point?
            x = point.x
            y = point.y
            z = point.z 
            inten = channel.values[0]
            found = False
            if z > 0.1:
                for i, pointDict in enumerate(self.bunches):
                    sX = pointDict['x']
                    sY = pointDict['y']
                    sZ = pointDict['z']
                    sN = pointDict['n']
                    dX = abs(abs(sX) - abs(x))
                    dY = abs(abs(sY) - abs(y))
                    dZ = abs(abs(sZ) - abs(z))
                    if dX < 0.1 and dY < 0.1 and dZ < 0.1:
                        self.bunches[i]['n'] += 1
                        if self.bunches[i]['i'] < inten:
                           self.bunches[i]['i'] = inten 
                        found = True
                if not found:
                    self.bunches.append({'x':x, 'y':y, 'z':z, 'n':1, 'i':inten })
        self.bunches = sorted(self.bunches, key=itemgetter('i'), reverse=False)


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