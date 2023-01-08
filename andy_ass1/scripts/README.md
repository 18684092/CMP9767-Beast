# PER18684092 CMP9767M-2223 Robot Programming Assignment

## Running the simulation

- roslaunch andy_ass1 ass1.launch

## Description

This solution uses a pre-made map of the environment along with a topological map to first localise the robot, using AMCL, taking approx. 3.5 minutes. The robot then navigates to the grapevine detecting and counting bunches of grapes using OpenCV and coloured contoured blob detection. It plots the bunches using a pointcloud within RVIZ and calculated the approx. yeild. 