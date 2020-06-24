#! /usr/bin/env python

"""
Reset script, where we reset all properties for simulation include physics and robot settings.
1. Creating and initializing a MoveGroupCommander object ("manipulator") as group. 
2. Define gazebo services.
3. Function: reset_function, resets the robot position and reset the simulation/world.
"""

import rospy
from std_srvs.srv import Trigger,Empty
import moveit_commander
from math import pi
from gazebo_msgs.srv import *

class ResetRobotSim (object):
    def __init__(self):
        rospy.init_node('reset_robot')   
        self.robot = moveit_commander.RobotCommander() 
        self.group = None
        while self.group == None:
            try:
                self.group = moveit_commander.MoveGroupCommander("manipulator")
            except Exception:
                rospy.sleep(1)

        #Call services for pause/unpause physics
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        #Servicese for reset simulation and world gazebo
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)     
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)   

        #Trigger for reset_f
        self.set_trigger = rospy.Service('reset_robot', Trigger, self.reset_f)    

    #JOINT[0]=-PI/2, JOINT[1]=-PI/2, JOINT[2]=0, JOINT[3]=0, JOINT[4]=0, JOINT[5]=0
    def reset_f(self, req):
        joint_target = {'shoulder_pan_joint' : -pi/2,
                        'shoulder_lift_joint': -pi/2,
                        'elbow_joint'       : 0,
                        'wrist_1_joint'     : 0,
                        'wrist_2_joint'     : 0,
                        'wrist_3_joint'     : 0 }
                                       
        self.group.set_joint_value_target(joint_target)
        self.group.go(wait=True)

        rospy.sleep(0.2)
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause.call()

        #rospy.wait_for_service('/gazebo/reset_simulation')
        #self.reset_simulation.call()

        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world.call()

        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause.call()

        return [True, 'succeded reset']

if __name__ == "__main__":
    rospy.wait_for_service('/move_group/get_loggers')
    resetrobotsim = ResetRobotSim()
    rospy.spin()
