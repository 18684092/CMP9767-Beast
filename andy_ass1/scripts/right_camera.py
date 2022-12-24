import rospy
import roslib, rospy, image_geometry, tf
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
import numpy as np


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
        self.camera = camera
        self.cv_image = None
        self.orig_image = None
        self.labeled_image = None
        self.color2depth_aspect = (84.1/1920) / (70.0/512)

        self.bunches = []
        # Initialize the CvBridge class
        self.bridge = CvBridge()
        # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
        sub_image = rospy.Subscriber("/thorvald_001/kinect2_" + self.camera + "_camera/hd/image_color_rect", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_' + camera + '_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/thorvald_001/kinect2_" + camera + "_sensor/sd/image_depth_rect", Image, self.image_depth_callback)
        self.tf_listener = tf.TransformListener()
        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseArray, queue_size=1)
        self.object_location_pub2 = rospy.Publisher('/thorvald_001/grapes_'+camera, PointCloud, queue_size=1)


    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    ##############
    # show_image #
    ##############
    # https://www.tutorialkart.com/opencv/python/opencv-python-resize-image/
    def show_image(self, img, name): 
        cv2.imshow(name, self.iResize(img))
        

    ###########
    # iResize #
    ###########
    def iResize(self, img):
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
        num_labels, labels = cv2.connectedComponents(img)
        # Map component labels to hue val, 0-179 is the hue range in OpenCV
        label_hue = np.uint8(179 * labels / np.max(labels))
        blank_ch = 255*np.ones_like(label_hue)
        self.labeled_image = cv2.merge([label_hue, blank_ch, blank_ch])
        # Converting cvt to BGR
        self.labeled_image = cv2.cvtColor(self.labeled_image, cv2.COLOR_HSV2BGR)
        # set bg label to black
        self.labeled_image[label_hue==0] = 0
        self.labeled_image = cv2.cvtColor(self.labeled_image, cv2.COLOR_BGR2RGB)
        return num_labels

    ################
    # getPositions #
    ################
    # Taken from https://stackoverflow.com/questions/56995532/opencv-blob-detector-isnt-detecting-white-blobs
    # Taken from https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
    def getPositions(self):
        image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        ps = PoseArray()
        pc = PointCloud()
        ps.header.frame_id = "map"
        pc.header.frame_id = "map"
        pc.header.stamp = rospy.Time.now()
        cnts = cv2.findContours(self.erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        min_area = 250
        i = 0
        for c in cnts:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            area = cv2.contourArea(c)
            if area > min_area:
                
                i += 1
                cv2.drawContours(self.orig_image, [c], -1, (0,0,255), 2)
                cv2.putText(self.orig_image, str(i), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                self.bunches.append(c)

                depth_coords = (image_depth.shape[0]/2 + (cY - self.orig_image.shape[0]/2)*self.color2depth_aspect, 
                    image_depth.shape[1]/2 + (cX - self.orig_image.shape[1]/2)*self.color2depth_aspect)
                try:
                    # get the depth reading at the centroid location
                    depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

                    #print('depth value: ', depth_value)
                    cv2.circle(self.orig_image, (cX, cY), 7, (255, 255, 255), -1)

                    # calculate object's 3d location in camera coords
                    camera_coords = self.camera_model.projectPixelTo3dRay((cX, cY)) #project the image coords (x,y) into 3D ray in camera coords 
                    camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                    camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

                    #print('camera coords: ', camera_coords)

                    #define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.frame_id = "thorvald_001/kinect2_" + self.camera + "_rgb_optical_frame"
                    object_location.pose.orientation.w = 1
                    
                    object_location.pose.position.x = camera_coords[0]
                    object_location.pose.position.y = camera_coords[1]
                    object_location.pose.position.z = camera_coords[2] 

                    try:
                        # print out the coordinates in the map frame
                        p_camera = self.tf_listener.transformPose('map', object_location)
                        
                        
                        #print(p_camera)
                        if "nan" not in str(p_camera.pose):
                            ps.poses.append(p_camera.pose)
                            p = Point()
                            p.x = p_camera.pose.position.x
                            p.y = p_camera.pose.position.y
                            p.z = p_camera.pose.position.z
                            pc.points.append(p)
                    except Exception as e:
                        print("ar crap", e)
                except:
                    cv2.circle(self.orig_image, (cX, cY), 7, (0, 0, 255), -1)
        self.object_location_pub.publish(ps)
        self.object_location_pub2.publish(pc)
        print("ps",pc)
        #print("original", self.orig_image.shape)
        #print("depth", image_depth.shape )

        return i

    ##################
    # image_callback #
    ##################
    def image_callback(self, img_msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        self.orig_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        blue_lower=np.array([0,2,50],np.uint8) 
        blue_upper=np.array([50,255,195],np.uint8)
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        res = cv2.bitwise_and(hsv, self.cv_image, mask=mask)
        kernel = np.ones((5,5),np.uint8)	
        kernel2 = np.ones((11,11),np.uint8)
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        gray = cv2.blur(gray, (5,5)	)
        (thresh, gray) = cv2.threshold(gray, 0, 198, cv2.THRESH_BINARY)
        dilation = cv2.dilate(gray,kernel,iterations = 1)
        self.erosion = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel2)
        
        i = self.getPositions()
        num_labels = self.connectedComponents(mask)
        try:
            print("Avg per bunch: ", int(num_labels/i) )
        except:
            pass

        self.showImages()

    ##############
    # showImages #
    ##############
    def showImages(self):
        #self.show_image(self.erosion, "Eroded")
        self.show_image(self.orig_image, self.camera)
        #self.show_image(self.labeled_image, "main")
        cv2.waitKey(1)




########
# main #
########
def main():
    rospy.init_node('bunches', anonymous=True)
    bunch = findBunches()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():     
        rate.sleep()

if __name__ == '__main__':
    main()