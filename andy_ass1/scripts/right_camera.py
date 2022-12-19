import rospy
from sensor_msgs.msg import Image
import numpy as np


# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Define a function to show the image in an OpenCV Window
def show_image(img):
    print("Showing image")
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

    scale_percent = 50 # percent of original size
    width = int(cv_image.shape[1] * scale_percent / 100)
    height = int(cv_image.shape[0] * scale_percent / 100)
    dim = (width, height)
    
    # resize image
    cv_image = cv2.resize(cv_image, dim, interpolation = cv2.INTER_AREA)

    #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    blue_lower=np.array([0,2,90],np.uint8)
    blue_upper=np.array([50,255,155],np.uint8)
    mask = cv2.inRange(hsv, blue_lower, blue_upper)
    res = cv2.bitwise_and(hsv, cv_image, mask=mask)
    show_image(mask)


    # Show the converted image
    

rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()



# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()