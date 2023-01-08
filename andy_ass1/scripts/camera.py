###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

import rospy
import roslib, rospy, image_geometry, tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud, ChannelFloat32
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Point32
import numpy as np
from std_msgs.msg import String
import json
import math

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

#########
# class #
#########
class findBunches:
    camera_model = None
    image_depth_ros = None


    ########
    # init #
    ########
    def __init__(self, camera = "right"):

        # So we known which camera we are
        self.camera = camera
        self.num_bunches = 0
        self.num_labels = 0
        self.iNum = 0

        self.moving = "true"
        self.row = ''
        self.rowMinMax = {"row1": {"min": 11, "max": -11}, "row2": {"min": 11, "max": -11}}
        # Stores various images
        self.cv_image = None
        self.orig_image = None
        self.labeled_image = None
        self.image_depth = None

        self.stampDepth = None


        # 1920 is width of HD colour, 512 is SD depth width
        # 84.1 and 70.0 taken from kinect2-gazebo.xacro file
        self.color2depth_aspect = (84.1/1920) / (70.0/512)

        # A list of x,y,z for each bunch detected
        self.bunches = []

        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # To find map position from Kinect2 x,y,depth info 
        self.tf_listener = tf.TransformListener()
    
        sub_image = rospy.Subscriber("/thorvald_001/kinect2_" + self.camera + "_camera/hd/image_color_rect", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_' + camera + '_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/thorvald_001/kinect2_" + camera + "_sensor/sd/image_depth_rect", Image, self.image_depth_callback)

        rospy.Subscriber("/thorvald_001/moving", String, self.callback_moving)
        rospy.Subscriber("/thorvald_001/row", String, self.callback_row)
        self.move = rospy.Publisher('/thorvald_001/camera_done', String, queue_size=1, latch=True)
        self.widths = rospy.Publisher('/thorvald_001/row_widths', String, queue_size=10, latch=True)
        
        # Publish a pointcloud - NOTE - intensities are own format and not compatible with RVIZ
        # but these get changed and made compatible by collate_point.py ready for RVIZ.
        self.object_location_pub2 = rospy.Publisher('/thorvald_001/grapes_'+camera, PointCloud, queue_size=1)

    def callback_moving(self, data):
        self.moving = data.data
       
    def callback_row(self, data):
        self.row = str(data.data)
       
    ########################
    # camera_info_callback #
    ########################
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    ########################
    # image_depth_callback #
    ########################
    def image_depth_callback(self, data):
        ''' Receives kinect2 depth data '''
        self.image_depth_ros = data
        self.stampDepth = data.header.stamp

    ##############
    # show_image #
    ##############
    def show_image(self, img, name):
        ''' Displays an image ''' 
        cv2.imshow(name, self.iResize(img))
        
    ###########
    # iResize #
    ###########
    # https://www.tutorialkart.com/opencv/python/opencv-python-resize-image/
    def iResize(self, img):
        ''' Resizes an image '''
        scale_percent = 50 
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        return img

    #######################
    # connectedComponents #
    #######################
    def connectedComponents(self, img):
        self.num_labels, labels = cv2.connectedComponents(img)
        # Map component labels to hue val, 0-179 is the hue range in OpenCV
        label_hue = np.uint8(179 * labels / np.max(labels))
        blank_ch = 255*np.ones_like(label_hue)
        self.labeled_image = cv2.merge([label_hue, blank_ch, blank_ch])
        # Converting cvt to BGR
        self.labeled_image = cv2.cvtColor(self.labeled_image, cv2.COLOR_HSV2BGR)
        # set bg label to black
        self.labeled_image[label_hue==0] = 0
        #self.labeled_image = cv2.cvtColor(self.labeled_image, cv2.COLOR_BGR2RGB)
        

    ################
    # getPositions #
    ################
    # Taken from https://stackoverflow.com/questions/56995532/opencv-blob-detector-isnt-detecting-white-blobs
    # Taken from https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
    # https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
    def getPositions(self):
        rMin = self.rowMinMax[self.row]['min']
        rMax = self.rowMinMax[self.row]['max']
        thisMin = 11
        thisMax = -11
        pc = PointCloud()
        pc.header.frame_id = "map"
        cnts = cv2.findContours(self.erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        min_area = 50
        self.num_bunches = 0
        for c in cnts:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            area = cv2.contourArea(c)
            if area > min_area:
                self.num_bunches += 1
                self.bunches.append(c)
                cv2.drawContours(self.orig_image, [c], -1, (0,0,255), 2)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(self.orig_image,(x-2,y-2),(x+w+2,y+h+2),(0,255,0),2)
                cv2.circle(self.orig_image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(self.orig_image, str(self.num_bunches), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                
                aX = []
                aY = []
                aZ = []
                try:
                    for i,v in enumerate([(-1,-1), (-1,0), (-1,1), (0, 0), (0, -1), (0, 1), (1,-1), (1,0), (1,1), (-2,-2), (-2,-1), (-2, 0), (-2,1), (-2,2), (-1,-2), (0,-2), (1,-2), (2,-2), (2,-1), (2,0), (2,1), (2,2), (1,2), (0,2),(-1,2), (-3,-3), (-3, -2), (-3,-1), (-3,0), (-3,1), (-3,2), (-3,3), (-2,-3), (-2,3), (-1,-3), (-1,3), (0,-3), (0,3),(1,-3), (1,3), (2,-3), (2,3), (3,-3),(3,-2),(3,-1),(3,0),(3,1),(3,2),(3,3)]):

                        # The depth array is smaller than the original colour image
                        depth_coords = (self.image_depth.shape[0] / 2 + ((cY+v[0]) - self.orig_image.shape[0] / 2) * self.color2depth_aspect, 
                            self.image_depth.shape[1] / 2 + ((cX+v[1]) - self.orig_image.shape[1] / 2) * self.color2depth_aspect)

                        if cY /2 > 511:
                            continue

                        # get the depth reading at the centroid location
                        depth_value = self.image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

                        # calculate object's 3d location in camera coords
                        camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
                        camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                        camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth
                        aX.append(camera_coords[0])
                        aY.append(camera_coords[1])
                        aZ.append(camera_coords[2])

                    print("deptg")
                    # any depth that is more than 0.2m away from average - delete
                    i = 0
                    while i < len(aZ):
                        if math.isnan(float(aZ[i])):
                            print("Odd Z", aX[i], aY[i], aZ[i])
                            del aZ[i]
                            del aX[i]
                            del aY[i]
                            print(len(aZ))
                        i += 1

                    avg = sum(aZ) / len(aZ)

                    xx = 0
                    yy = 0
                    zz = 0
                    tt = 0
                    print("Bunch", self.num_bunches)
                    for v in zip(aX, aY, aZ):
                        if math.isnan(float(v[0])) or math.isnan(float(v[1])) or math.isnan(float(v[2])):
                            print("bad", v[0], v[1], v[2])
                            continue
                        if abs(v[2]) > abs(avg) + 0.20:
                            print("bad extreme", v[0], v[1], v[2])
                            continue
                        tt += 1
                        xx += v[0]
                        yy += v[1]
                        zz += v[2]
                        print("good", v[0], v[1], v[2])
                    print()

                    #define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.stamp = self.stampDepth
                    object_location.header.frame_id = self.camera_model.tfFrame() #"thorvald_001/kinect2_" + self.camera + "_rgb_optical_frame"
                    object_location.pose.orientation.w = 1
                    object_location.pose.position.x = xx / tt
                    object_location.pose.position.y = yy / tt
                    object_location.pose.position.z = zz / tt

                    # get the coordinates in the map frame
                    p_camera = self.tf_listener.transformPose('map', object_location)

                    # Depth can get confused by blocks / objects close to camera
                    xL = p_camera.pose.position.y < -6 and p_camera.pose.position.y > -8.5

                    if "nan" not in str(p_camera.pose) and xL:
                        p = Point32()
                        ch = ChannelFloat32()
                        p.x = p_camera.pose.position.x
                        p.y = p_camera.pose.position.y
                        p.z = p_camera.pose.position.z
                        mini = p.x < rMin
                        maxi = p.x > rMax 
                        if mini or maxi:
                            if mini:
                                if p.x < thisMin:
                                    thisMin = p.x 
                            if maxi:
                                if p.x > thisMax:
                                    thisMax = p.x 
                            pc.points.append(p)
                            ch.name = "intensity"
                            ch.values = (area , area , area)
                            pc.channels.append(ch)


                # If point is out of bounds (no depth info) display it as red dot        
                except Exception as e:
                    print("exception", e, "bunch")
                    cv2.circle(self.orig_image, (cX, cY), 7, (0, 0, 255), -1)
        # Publish the point cloud
        self.object_location_pub2.publish(pc)
        if thisMin < self.rowMinMax[self.row]['min']:
            self.rowMinMax[self.row]['min'] = thisMin + 0.3
        if thisMax > self.rowMinMax[self.row]['max']:
            self.rowMinMax[self.row]['max'] = thisMax - 0.3  
        self.widths.publish(String(json.dumps(self.rowMinMax)))            

    ##################
    # image_callback #
    ##################
    def image_callback(self, img_msg):
        ''' Find grapes which are of a particular hue,
        produce a mask, blur slightly, dilate then erode.
        Find contours of blobs, find centroids of contours,
        publish their position with respect to map.
        '''

        if self.moving == "true": return

        self.move.publish(String('imaging'))

        # Image is BGR
        self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        self.image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        # We want to display a nice colour correct image
        self.orig_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

        # Colour format will change blue (ish) grapes to brown (ish) but we don't care
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        blue_lower=np.array([0,2,50],np.uint8) 
        blue_upper=np.array([50,255,195],np.uint8)

        # The mask is used to find contours / blobs
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        
        # The result shows individual grapes
        res = cv2.bitwise_and(hsv, self.cv_image, mask=mask)

        # Used as params for erosion and dilation
        kernel = np.ones((5,5),np.uint8) # was 5	
        kernel2 = np.ones((7,7),np.uint8) # was 11

        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # Blurring seems to make detection more robust
        gray = cv2.blur(gray, (3,3)	) # was 5
        (thresh, gray) = cv2.threshold(gray, 0, 198, cv2.THRESH_BINARY)
        dilation = cv2.dilate(gray, kernel, iterations = 1)
        self.erosion = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel2)
        
        # Uses contours to get grape bunches
        self.getPositions()

        # Returns number of grape within mask image
        self.connectedComponents(mask)

        # This try is for debugging
        try:
            print("Avg per bunch: ", int(self.num_labels/self.num_bunches) )
        except:
            pass
        
        self.move.publish(String('not imaging'))
        self.showImages()
        self.iNum += 1



    ##############
    # showImages #
    ##############
    def showImages(self):
        
        if self.num_bunches > 0:
            self.show_image(self.orig_image, self.camera)
            cv2.imwrite("/home/ubuntu/ros_ws/src/andy_ass1/images/"+self.row+"_"+str(self.iNum)+"_"+self.camera+".jpg", self.orig_image)
            #self.show_image(self.erosion, "Eroded")
        if self.num_labels > 20:
            self.show_image(self.labeled_image, self.camera + " Grapes")
        cv2.waitKey(25)

########
# main #
########
def main():
    rospy.init_node('camera_processing', anonymous=True)
    try:
        camera = rospy.get_param('~camera')
    except:
        camera = 'left' 
    bunch = findBunches(camera)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():     
        rate.sleep()

if __name__ == '__main__':
    main()