###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
################################### 

import rospy

# Not all of these are used - NOTE - TODO clean up
from sensor_msgs.msg import PointCloud, PointCloud2, ChannelFloat32, PointField
from geometry_msgs.msg import  Point, Point32

# Used for sorting
from operator import itemgetter

###########
# Collate #
###########
class Collate:
    ''' Collate all point clouds from cameras that detect grapes together '''

    ########
    # init #
    ########
    def __init__(self):

        # Store for all points from all cameras
        self.bunches = []
        # This will be published for RVIZ
        self.pcBunches = PointCloud()

        # The cameras
        left = rospy.Subscriber('/thorvald_001/grapes_left', PointCloud, self.left_callback)
        right = rospy.Subscriber('/thorvald_001/grapes_right', PointCloud, self.right_callback)
        front = rospy.Subscriber('/thorvald_001/grapes_front', PointCloud, self.front_callback)

        # The published point cloud with intensities
        self.grapeBunches = rospy.Publisher('/thorvald_001/grape_bunches', PointCloud, queue_size=30, latch=False)

    # Multiple cameras will publish a point cloud
    #################
    # left_callback #
    ################# 
    def left_callback(self, data):
        self.collate_points(data)

    ##################
    # right_callback #
    ##################
    def right_callback(self, data):
        self.collate_points(data)       

    ##################
    # front_callback #
    ##################
    def front_callback(self, data):
        self.collate_points(data)

    ###########
    # publish #
    ###########
    def publish(self):
        ''' Publish a proper point cloud with intensities '''
        pcBunches = PointCloud()
        pcBunches.header.frame_id = "map"
        pcBunches.header.stamp = rospy.Time.now()
        intensities = []
        for i, pointDict in enumerate(self.bunches):
            if i > 400: break
            # Each point must have been seen x times to show its a reliable point - (needs to be a particle filter)
            if pointDict['n'] > 0:
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
       
    ##################
    # collate_points #
    ##################
    def collate_points(self, pc):
        ''' Each point cloud will get collated together into one array of dictionaries '''
        for point,channel in zip(pc.points, pc.channels):
            # Is point within x cm of stored 3D point?
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
                    # Different between this point and a previously seen point
                    dX = abs(abs(sX) - abs(x))
                    dY = abs(abs(sY) - abs(y))
                    dZ = abs(abs(sZ) - abs(z))
                    # Crude filter to group noisy point clouds - not effective really
                    if dX < 0.3 and dY < 0.3 and dZ < 0.3:
                        # Multiple points that match suggest a better reading
                        self.bunches[i]['n'] += 1
                        # If this point is close enough to where another point is
                        # update the intensity value if bigger 
                        if self.bunches[i]['i'] < inten:
                           self.bunches[i]['i'] = inten 
                        found = True
                if not found:
                    # Add this new point
                    self.bunches.append({'x':x, 'y':y, 'z':z, 'n':1, 'i':inten })
        # Sorting them just in case we only want to publish the first or biggest x number of bunches
        self.bunches = sorted(self.bunches, key=itemgetter('n', 'i'), reverse=True)
        print(self.bunches)
        print()


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