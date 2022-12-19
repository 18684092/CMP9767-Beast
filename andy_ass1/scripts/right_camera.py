import rospy
from sensor_msgs.msg import Image
import numpy as np


# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Define a function to show the image in an OpenCV Window
def show_image(img, name):
    print("Showing image")
    cv2.imshow(name, img)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    
    cv_image2 = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    
    #scale_percent = 50 # percent of original size
    #width = int(cv_image.shape[1] * scale_percent / 100)
    #height = int(cv_image.shape[0] * scale_percent / 100)
    #dim = (width, height)
    
    # resize image
    #cv_image2 = cv2.resize(cv_image2, dim, interpolation = cv2.INTER_AREA)
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    blue_lower=np.array([0,2,50],np.uint8) # 80 was 90
    blue_upper=np.array([50,255,195],np.uint8)
    mask = cv2.inRange(hsv, blue_lower, blue_upper)
    res = cv2.bitwise_and(hsv, cv_image, mask=mask)

    show_image(mask, "main")

    kernel = np.ones((5,5),np.uint8)	
    kernel2 = np.ones((11,11),np.uint8)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray, (5,5)	)
    (thresh, gray) = cv2.threshold(gray, 0, 198, cv2.THRESH_BINARY)
    dilation = cv2.dilate(gray,kernel,iterations = 1)
    dilation = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel2)

    show_image(dilation, "dilation")
    cnts = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    min_area = 550
    white_dots = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area:
            cv2.drawContours(cv_image2, [c], -1, (0,0,255), 3)
            white_dots.append(c)

    print(len(white_dots))



    show_image(cv_image2, "orig")

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