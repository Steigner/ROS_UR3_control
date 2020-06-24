#! /usr/bin/env python

"""
Script for publishing current position by rospy node. 
1. Initializing the moveit_commander module and ROS node.
2. Creating and initializing MoveGroupCommander object ("manipulator") as group. 
3. Setup publisher for publishing current position.
"""

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('current_pose_publisher', anonymous=True)
group = moveit_commander.MoveGroupCommander("manipulator")
publisher = rospy.Publisher("current_position", geometry_msgs.msg.PoseStamped, queue_size=5)

if __name__ == "__main__":
    #Get current position and publish it 
    while not rospy.is_shutdown():
        current_position = group.get_current_pose()
        publisher.publish(current_position)

    
