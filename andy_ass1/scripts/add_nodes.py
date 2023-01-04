import rospy

from topological_navigation_msgs.srv import AddNode
from geometry_msgs.msg import Pose
import subprocess

if __name__ == '__main__':
    rospy.wait_for_service('topological_map_manager2/add_topological_node')
    add_node_service = rospy.ServiceProxy('/topological_map_manager2/add_topological_node', AddNode)

    try:
        pose = Pose()
        pose.position.x = -11.0
        pose.position.y = 11.0
        pose.orientation.w = 1.0
        response = add_node_service("waypoint101", pose, False)
        print(response.success)
    except: 
        print("Failed")

 
    a = subprocess.run(["/opt/ros/noetic/bin/rosservice", "call", "/topological_map_manager2/add_edges_between_nodes", "{origin: 'waypoint101', destination: 'waypoint1', action: 'move_base', edge_id: 'waypoint101_waypoint1'}"])
    print(a)         
