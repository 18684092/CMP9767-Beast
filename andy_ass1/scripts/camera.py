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
    def __init__(self, camera = "right", area = 100):

        # So we known which camera we are
        self.camera = camera
        self.num_bunches = 0
        self.num_labels = 0
        self.iNum = 0

        # Rather arbitary area, small enough to catch almost totally occluded bunches, big enough not to see rouge pixel colours as a bunch 
        self.minArea = area

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
        # NOTE TODO the 70.0 figure is disputed and should be 69.x or something
        # https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9002889/#:~:text=The%20Kinect%20v2%20depth%20sensor,4.5%20m%20range%20%5B13%5D.
        self.color2depth_aspect = (84.1/1920) / (70.0/512)

        # A list of x,y,z for each bunch detected
        self.bunches = []

        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # To find map position from Kinect2 x,y,depth info 
        self.tf_listener = tf.TransformListener()
    
        # The main camera image   
        sub_image = rospy.Subscriber("/thorvald_001/kinect2_" + self.camera + "_camera/hd/image_color_rect", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_' + self.camera + '_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/thorvald_001/kinect2_" + self.camera + "_sensor/sd/image_depth_rect", Image, self.image_depth_callback)
        rospy.Subscriber("/thorvald_001/moving", String, self.callback_moving)
        rospy.Subscriber("/thorvald_001/row", String, self.callback_row)

        # Go_grapevine.py needs to know when camera image processing has finished 
        self.move = rospy.Publisher('/thorvald_001/camera_done', String, queue_size=1, latch=True)

        # Row widths used within main_display.py 
        self.widths = rospy.Publisher('/thorvald_001/row_widths', String, queue_size=10, latch=True)
        
        # Publish a pointcloud. 
        # but these get changed and made compatible by point_colation.py ready for RVIZ.
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
        
        # Map component labels to hue val, 0-179 is the hue range in OpenCV - taken from tutorial code
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

        # Extremities of grapevine seen so far
        rMin = self.rowMinMax[self.row]['min']
        rMax = self.rowMinMax[self.row]['max']
        thisMin = 11
        thisMax = -11

        # Setup the point cloud
        pc = PointCloud()
        pc.header.frame_id = "map"

        # Find contours of blobs
        cnts = cv2.findContours(self.erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # Extremity flag
        extremity = False      

        #Draw contours, boxes and centroids
        self.num_bunches = 0
        for c in cnts:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            # Area is used for intensity values for point clouds
            area = cv2.contourArea(c)
            if area > self.minArea:
                self.num_bunches += 1
                print("Bunch", self.num_bunches)
                self.bunches.append(c)
                cv2.drawContours(self.orig_image, [c], -1, (0,0,255), 2)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(self.orig_image,(x-2,y-2),(x+w+2,y+h+2),(0,255,0),2)
                cv2.circle(self.orig_image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(self.orig_image, str(self.num_bunches), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                #cv2.putText(self.orig_image, str(int(area)), (x , y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # Rather than use a single pixel point an area is sampled for depth and to reduce x,y positional noise
                aX = []
                aY = []
                aZ = []
                try:
                    # OK - so this could have been a 2D array auto generated but I was experimenting and making different patterns to see if it was more efficient :)
                    for i,v in enumerate([(-1,-1), (-1,0), (-1,1), (0, 0), (0, -1), (0, 1), (1,-1), (1,0), (1,1), (-2,-2), (-2,-1), (-2, 0), (-2,1), (-2,2), (-1,-2), (0,-2), (1,-2), (2,-2), (2,-1), (2,0), (2,1), (2,2), (1,2), (0,2),(-1,2), (-3,-3), (-3, -2), (-3,-1), (-3,0), (-3,1), (-3,2), (-3,3), (-2,-3), (-2,3), (-1,-3), (-1,3), (0,-3), (0,3),(1,-3), (1,3), (2,-3), (2,3), (3,-3),(3,-2),(3,-1),(3,0),(3,1),(3,2),(3,3),
                     (-4,-4), (-4,-3), (-4,-2), (-4,-1), (-4,0), (-4,1), (-4,2), (-4,3), (4,-4), (4,-3), (4,-2), (4,-1), (4,0), (4,1), (4,2), (4,3), (4,4)]):

                        # The depth array is smaller than the original colour image
                        depth_coords = (self.image_depth.shape[0] / 2 + ((cY+v[0]) - self.orig_image.shape[0] / 2) * self.color2depth_aspect, 
                            self.image_depth.shape[1] / 2 + ((cX+v[1]) - self.orig_image.shape[1] / 2) * self.color2depth_aspect)

                        # get the depth reading at the centroid location
                        # Any failure here gets caught on the "except:" - which draws a red circle on the bunch to highlight a failure
                        depth_value = self.image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

                        # calculate object's 3d location in camera coords
                        camera_coords = self.camera_model.projectPixelTo3dRay((cX+v[1], cY+v[0])) #project the image coords (x,y) into 3D ray in camera coords 
                        camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                        camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth
                        
                        aX.append(camera_coords[0])
                        aY.append(camera_coords[1])
                        aZ.append(camera_coords[2])

                    ##################################################################################################
                    # EXPERIMENTING!!!                                                                               #
                    ##################################################################################################
                    # any depth that is more than 0.2m away from average - delete
                    i,s,c = 0,0,0
                    while i < len(aZ):
                        if not math.isnan(float(aZ[i])):
                            c += 1   
                            s += aZ[i]
                        print(i, "Z = nan")
                        i += 1
                    # If c is 0 then except will catch and draw a red dot
                    avg = s / c
                    print("Avg:", avg, "count: ", c)

                    xx,yy,zz,tt = 0,0,0,0
                    for v in zip(aX, aY, aZ):
                        if math.isnan(float(v[0])) or math.isnan(float(v[1])) or math.isnan(float(v[2])):
                            continue
                        if abs(v[2]) > abs(avg) + 0.20:
                            print("Extreme depth - x: ", v[0], "y: ", v[1], "z: ", v[2])
                            continue
                        tt += 1
                        xx += v[0]
                        yy += v[1]
                        zz += v[2]
                        print("x: ", v[0], "y: ", v[1], "z: ", v[2])
                    print()
                    ##################################################################################################


                    #define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.stamp = self.stampDepth # Makes no difference if included - TODO why?
                    object_location.header.frame_id = self.camera_model.tfFrame() 
                    object_location.pose.orientation.w = 1
                    
                    # Produce an average location
                    object_location.pose.position.x = round(xx / tt, 3)
                    object_location.pose.position.y = round(yy / tt, 3)
                    object_location.pose.position.z = round(zz / tt, 3)

                    # get the coordinates in the map frame
                    p_camera = self.tf_listener.transformPose('map', object_location)

                    print("Average location and depth: ", round(xx / tt, 3), round(yy / tt, 3), round(zz / tt, 3))

                    # Depth can get confused by blocks / objects close to camera
                    # NOTE TODO this should be changed as distance away from camera not a map position
                    xL = p_camera.pose.position.y < -6.5 and p_camera.pose.position.y > -8.5

                    # we have a good bunch 
                    if "nan" not in str(p_camera.pose) and xL:
                        p = Point32()
                        ch = ChannelFloat32()
                        p.x = p_camera.pose.position.x
                        p.y = p_camera.pose.position.y
                        p.z = p_camera.pose.position.z

                        # Keep track of the extremeties of bunches so as to not double count
                        mini = p.x < rMin
                        maxi = p.x > rMax 

                        # Keep track of furthest bunch seen so far
                        if mini or maxi:
                            if mini:
                                if p.x < thisMin:
                                    thisMin = p.x
                            if maxi:
                                if p.x > thisMax:
                                    thisMax = p.x 

                            # this bunch is outside of what has previously been seen so append it to the pointcloud
                            pc.points.append(p)
                            ch.name = "intensity"
                            ch.values = (area , area , area)
                            pc.channels.append(ch)

                # If point is out of bounds (no depth info) display it as red dot        
                except Exception as e:
                    cv2.circle(self.orig_image, (cX, cY), 7, (0, 0, 255), -1)

        # Publish the point cloud
        self.object_location_pub2.publish(pc)

        # The next image will only count bunches not seen before BUT allow some over lap
        # Hopefully double counting is picked up within point_colation.py
        if thisMin < self.rowMinMax[self.row]['min']:
            self.rowMinMax[self.row]['min'] = thisMin + 0.3
        if thisMax > self.rowMinMax[self.row]['max']:
            self.rowMinMax[self.row]['max'] = thisMax - 0.3  
        self.widths.publish(String(json.dumps(self.rowMinMax)))    
               
        # Draw lines showing the valid bunches for this image that haven't been counted before
        #cv2.line(self.orig_image, (int(self.map3DtoPixel(minBunch)[0]), 0), (int(self.map3DtoPixel(minBunch)[0]), self.orig_image.shape[0]-1), (0,255,255), 2)
        #cv2.line(self.orig_image, (int(self.map3DtoPixel(maxBunch)[0]), 0), (int(self.map3DtoPixel(maxBunch)[0]), self.orig_image.shape[0]-1), (0,255,255), 2)

    ################
    # map3DtoPixel #
    ################
    # Map a 3D x,y,z coord to image pixel
    def map3DtoPixel(self, p):
        ''' Project point p back to image '''
        o = PoseStamped()
        o.header.stamp = self.stampDepth
        o.header.frame_id = "map" 
        o.pose.orientation.w = 1
        
        # Produce an average location
        o.pose.position.x = p.x
        o.pose.position.y = p.y
        o.pose.position.z = p.z

        # get the coordinates in the map frame
        p_camera = self.tf_listener.transformPose("thorvald_001/kinect2_left_rgb_optical_frame", o)
        uv = self.camera_model.project3dToPixel((p_camera.pose.position.x, p_camera.pose.position.y, p_camera.pose.position.z))                 
        return uv

    ##################
    # image_callback #
    ##################
    def image_callback(self, img_msg):
        ''' Find grapes which are of a particular hue,
        produce a mask, blur slightly, dilate then erode.
        Find contours of blobs, find centroids of contours,
        publish their position with respect to map.
        '''

        # Do nothing if robot still moving
        if self.moving == "true": return

        # The the main navigation node know the camera is busy
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
        
        # Notify control node that camera has finished
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
            cv2.imwrite("/home/ubuntu/ros_ws/src/andy_ass1/images/"+self.row+"_grapes_"+str(self.iNum)+"_"+self.camera+".jpg", self.labeled_image)
            #self.show_image(self.labeled_image, self.camera + " Grapes")
        cv2.waitKey(25)

########
# main #
########
def main():
    rospy.init_node('camera_processing', anonymous=True)
    try:
        camera = rospy.get_param('~camera')
        area = int(rospy.get_param('~area'))
    except:
        camera = 'left' 
        area = 100
    bunch = findBunches(camera, area)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():     
        rate.sleep()

if __name__ == '__main__':
    main()