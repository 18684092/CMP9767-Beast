# PER18684092 CMP9767M-2223 Robot Programming Assignment

## Counting Bunches Of Grapes

## Summary Of Solution

This solution uses a pre-made map of the environment along with a topological map to first localise the robot, using AMCL, allowing the robot to scan objects in all directions, taking approx. 3.5 minutes. The robot then navigates to the grapevine detecting and counting bunches of grapes using camera imaging, OpenCV and coloured contoured blob detection. It plots the detected bunches using a pointcloud with locations determined by a depth camera and transformed to the map position within RVIZ. The yield is calculated allowing an approximation for any size field. 

## Running The Simulation

- Install docker and the image as per [CMP9767 Wiki Page](https://github.com/LCAS/CMP9767M/wiki/using-the-module-resources-in-docker).
- run `docker-compose up`
- roslaunch andy_ass1 ass1.launch

## Individual Components and Algorithms Used

- **Object detection** within script object_distance.py. **Publishes a String** to topic '/thorvald_001/object_distance' (serialised dictionary) detailing closest object in front, back, left and right of robot.
- **Object position** within script object_distance_tf.py. **Publishes a PoseStamped** to topic '/thorvald_001/nearest_obstacle_pose' detailing the position of closest object in relation to front and back scanner.
- **Moving the robot** within script open_move.py. Uses above object detection to avoid objects while moving randomly around a world. Turns randomly in either 'forward', 'back', 'left' or 'right' directions. Keeps in a straight line. Used to experiment with AMCL and understanding laser scans.
- Real time adding of **nodes and edges** to a topological map using script **add_nodes.py** while a simulation is running. Uses 'topological_map_manager2/add_topological_node' and '/topological_map_manager2/add_edges_between_nodes' as service calls. This can be seen within the solution as 20 seconds into launching extra nodes and edges appear.
- **Action library** setting navigation goals with GotoNodeAction within script **go_grapevine.py**. This script controls all navigation and publishes '/thorvald_001/row', '/thorvald_001/state' and '/thorvald_001/moving' for other scripts (camera.py and main_display.py) to know when and what is going on. This directly controls when the robot stops and tells the camera when and where to take images. It also subscribes to the topic '/thorvald_001/camera_done' so as to wait until image processing is complete and the next part of the grapevine can be imaged.
- **Pointclouds** within script **point_colation.py**. Receives point clouds on topics '/thorvald_001/grapes_left', '/thorvald_001/grapes_right' and '/thorvald_001/grapes_front' for all 3 cameras (there can be 3 instances of camera.py running). The 'upto' 3 pointclouds are combined into one published pointcloud '/thorvald_001/grape_bunches' for displaying within RVIZ. This pointcloud uses intensity to show different grapebunch size. With noisy (x,y,z) map positions this script will aggrigate multiple bunches into 1 bunch if a bunch location is within 0.2m cubed of another bunch. This is a form of 'binning' and acts as a noise filter.
- **Image**, **CameraInfo**, **OpenCV**, **FindContours**, **Moments**, **ContourArea**, **projectPixelTo3dRay**, **Image Scaling** and **tf_listener** implemented within script **camera.py**. This script detects grapebunches, saves images and publishes pointclouds of bunch positions. and takes launch file parameters to configure which camera will be used using **rospy.get_param()**.
- **OpenCV** image creation in the form of a display screen within script **main_display.py**. This script feedsback to the user what is going on and final results. 

