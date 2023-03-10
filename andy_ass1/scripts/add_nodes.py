###################################
# Author: Andy Perrett (18684092) #
# Date  : 27th December 2022      #
# Module: CMP9767                 #
###################################

# Rough and ready taken from lecture 9 slide code, although node array added and
# edges added. TODO edges need to be inserted within node list but subprocess is
# quick solution for now.

import rospy
import time
from topological_navigation_msgs.srv import AddNode
from geometry_msgs.msg import Pose
import subprocess

if __name__ == '__main__':

    # deliberately delay so we can see nodes being added within RVIZ
    time.sleep(20)

    # The list of nodes (x,y)
    nodes = [(-11,11),(-11,8), (-11,4), (-7,8), (-7,4), (-7.5, -2)]

    # Wait then add
    rospy.wait_for_service('topological_map_manager2/add_topological_node')
    add_node_service = rospy.ServiceProxy('/topological_map_manager2/add_topological_node', AddNode)

    # Loop can fail during testing if duplicate nodes added
    try:
        for i, p in enumerate(nodes):
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.orientation.w = 1.0
            response = add_node_service("waypoint10"+str(i+1), pose, False)

    except: 
        print("Failed")

    # Nasty but what the hell! Just demonstrating we can add edges 
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint1', destination: 'waypoint101', action: 'move_base', edge_id: 'waypoint1_waypoint101'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint101', destination: 'waypoint1', action: 'move_base', edge_id: 'waypoint101_waypoint1'}"])      

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint101', destination: 'waypoint102', action: 'move_base', edge_id: 'waypoint101_waypoint102'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint102', destination: 'waypoint101', action: 'move_base', edge_id: 'waypoint102_waypoint101'}"])   

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint102', destination: 'waypoint103', action: 'move_base', edge_id: 'waypoint102_waypoint103'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint103', destination: 'waypoint102', action: 'move_base', edge_id: 'waypoint103_waypoint102'}"]) 

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint103', destination: 'waypoint104', action: 'move_base', edge_id: 'waypoint103_waypoint104'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint104', destination: 'waypoint103', action: 'move_base', edge_id: 'waypoint104_waypoint103'}"]) 

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint104', destination: 'waypoint105', action: 'move_base', edge_id: 'waypoint104_waypoint105'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint105', destination: 'waypoint104', action: 'move_base', edge_id: 'waypoint105_waypoint104'}"]) 

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint105', destination: 'waypoint103', action: 'move_base', edge_id: 'waypoint105_waypoint103'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint103', destination: 'waypoint105', action: 'move_base', edge_id: 'waypoint103_waypoint105'}"]) 

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint105', destination: 'waypoint106', action: 'move_base', edge_id: 'waypoint105_waypoint106'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint106', destination: 'waypoint105', action: 'move_base', edge_id: 'waypoint106_waypoint105'}"]) 

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint19', destination: 'waypoint106', action: 'move_base', edge_id: 'waypoint19_waypoint106'}"])

    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint105', destination: 'WayPoint0', action: 'move_base', edge_id: 'waypoint105_WayPoint0'}"])
    subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'WayPoint0', destination: 'waypoint105', action: 'move_base', edge_id: 'WayPoint0_waypoint105'}"]) 
