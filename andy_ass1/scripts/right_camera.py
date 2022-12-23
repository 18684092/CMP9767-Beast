import rospy
from sensor_msgs.msg import Image
import numpy as np


# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

#########
# class #
#########
class findBunches:

    ########
    # init #
    ########
    def __init__(self, camera = "right"):
        self.camera = camera
        self.cv_image = None
        self.orig_image = None
        self.labeled_image = None
        self.bunches = []
        # Initialize the CvBridge class
        self.bridge = CvBridge()
        # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
        sub_image = rospy.Subscriber("/thorvald_001/kinect2_" + self.camera + "_camera/hd/image_color_rect", Image, self.image_callback)

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
                cv2.circle(self.orig_image, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(self.orig_image, str(i), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                self.bunches.append(c)
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
        self.show_image(self.erosion, "Eroded")
        self.show_image(self.orig_image, "orig")
        self.show_image(self.labeled_image, "main")
        cv2.waitKey(1)


# Define a function to show the image in an OpenCV Window
def show_image(img, name):
    #print("Showing image")
    cv2.imshow(name, img)
    cv2.waitKey(3)

# Define a callback for the Image message
# def image_callback(img_msg):

#     cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     cv_image2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    

    
#     hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#     blue_lower=np.array([0,2,50],np.uint8) 
#     blue_upper=np.array([50,255,195],np.uint8)
#     mask = cv2.inRange(hsv, blue_lower, blue_upper)
#     res = cv2.bitwise_and(hsv, cv_image, mask=mask)

#     num_labels, labels = cv2.connectedComponents(mask)
    
#     # Map component labels to hue val, 0-179 is the hue range in OpenCV
#     label_hue = np.uint8(179*labels/np.max(labels))
#     blank_ch = 255*np.ones_like(label_hue)
#     labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

#     # Converting cvt to BGR
#     labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

#     # set bg label to black
#     labeled_img[label_hue==0] = 0
#     labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_BGR2RGB)
#     show_image(labeled_img, "main")

#     kernel = np.ones((5,5),np.uint8)	
#     kernel2 = np.ones((11,11),np.uint8)
#     gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
#     gray = cv2.blur(gray, (5,5)	)
#     (thresh, gray) = cv2.threshold(gray, 0, 198, cv2.THRESH_BINARY)
#     dilation = cv2.dilate(gray,kernel,iterations = 1)
#     erosion = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel2)

#     show_image(erosion, "Eroded")
#     cnts = cv2.findContours(erosion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     cnts = cnts[0] if len(cnts) == 2 else cnts[1]
#     min_area = 250
#     bunches = []

#     # Taken from https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
#     # Taken from https://stackoverflow.com/questions/56995532/opencv-blob-detector-isnt-detecting-white-blobs
#     i = 0
#     for c in cnts:
#         M = cv2.moments(c)
#         cX = int(M["m10"] / M["m00"])
#         cY = int(M["m01"] / M["m00"])
#         area = cv2.contourArea(c)
#         if area > min_area:
#             i += 1
#             cv2.drawContours(cv_image2, [c], -1, (0,0,255), 2)
#             cv2.circle(cv_image2, (cX, cY), 7, (255, 255, 255), -1)
#             cv2.putText(cv_image2, str(i), (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
#             bunches.append(c)

#     print(i, num_labels)
#     try:
#         print("Avg per bunch: ", int(num_labels/i) )
#     except:
#         pass


#     show_image(cv_image2, "orig")

    # Show the converted image
    

#rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
#rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
#bridge = CvBridge()

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
#sub_image = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
#cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
#while not rospy.is_shutdown():
#    rospy.spin()

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