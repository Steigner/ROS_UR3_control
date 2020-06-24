#! /usr/bin/env python

"""
Script where we get action data for move. 
1. Function: data_action get from script: ~/testos/src/random_target_reaching.py from function:step, action.data, which direction robot should move.
In function is condition tree, which direction in axis should robot move.
2. From function: data_action, get function: move, position and than robot go to position.
3. Subscriber: action.data in span <0;5> Integer
"""

import sys
import rospy
import moveit_commander
import std_msgs
import geometry_msgs.msg
import rospkg

def data_action(action):
    #Set each step size for move
    step_size = 0.005
    current_position = move_group.get_current_pose()
    current_p_joints = move_group.get_current_joint_values()

    with open (path,'a') as file:           
        file.write(str (current_p_joints[0]) + "," + str (current_p_joints[1]) + "," + str (current_p_joints[2]) + "," + str (current_p_joints[3]) + "," + str (current_p_joints[4]) + "," + str (current_p_joints[5]) + "\n")
    file.close()

    #ACTION MOVE: 0 = +X AXIS, ACTION MOVE: 1 = -X AXIS, ACTION MOVE: 2 = +Y AXIS, ACTION MOVE: 3 = -Y AXIS, ACTION MOVE: 4 = +Z AXIS, ACTION MOVE: 5 = -Z AXIS
    if action.data == 0:
        current_position.pose.position.x = current_position.pose.position.x + step_size
        move_f(current_position)
    elif action.data == 1:
        current_position.pose.position.x = current_position.pose.position.x - step_size
        move_f(current_position)
    elif action.data == 2:
        current_position.pose.position.y = current_position.pose.position.y + step_size
        move_f(current_position)
    elif action.data == 3:
        current_position.pose.position.y = current_position.pose.position.y - step_size
        move_f(current_position)
    elif action.data == 4:
        current_position.pose.position.z = current_position.pose.position.z + step_size
        move_f(current_position)
    elif action.data == 5:
        current_position.pose.position.z = current_position.pose.position.z - step_size
        move_f(current_position)
    return current_position

#Move function to get position target
def move_f(position_target):
    move_group.set_pose_target(position_target)
    plan = move_group.plan()
    move_group.go(wait=True) 

    move_group.stop()
    move_group.clear_pose_targets() 

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_robot', anonymous=True)
    move_group = moveit_commander.MoveGroupCommander("manipulator")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('testos')

    pathq_joints = '/src/traj_qjoints'
    paths_joints = '/src/traj_sjoints'
    pathdqn_joints = '/src/traj_dqnjoints'
    pathdsarsa_joints = '/src/traj_dsarsajoints'
    
    path = pkg_path + pathq_joints

    while not rospy.is_shutdown():
        #Subscriber: data_action
        rospy.Subscriber("action_move_data", std_msgs.msg.Int8, data_action)
        rospy.spin()

