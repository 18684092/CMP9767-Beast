# PER18684092 CMP9767M-2223 Robot Programming Assignment

## Counting of bunches of grapes

## Summary of solution

This solution uses a pre-made map of the environment along with a topological map to first localise the robot, using AMCL, allowing the robot to scan objects in all directions, taking approx. 3.5 minutes. The robot then navigates to the grapevine detecting and counting bunches of grapes using camera imaging, OpenCV and coloured contoured blob detection. It plots the detected bunches using a pointcloud with locations determined by a depth camera and transformed to the map position within RVIZ. The yield is calculated allowing an approximation for any size field. 

## Running the simulation

- roslaunch andy_ass1 ass1.launch

## 

