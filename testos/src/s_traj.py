#!/usr/bin/env python

import sys
import rospy
import time

import moveit_commander
import geometry_msgs.msg 

import rospkg
import numpy as np
import show

from NeoPySwitch import SwitchCase as case
from NeoPySwitch import PySwitch as switch

from math import pi
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class Target_simulation(object):
  def __init__(self):
  # -------------------------------INICIALIZATION -----------------------------
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('TESTOS', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
  
    self.box_name = ''
    self.scene = scene
    self.robot = robot
    self.move_group = move_group
  # ---------------------------------------------------------
  def init_motion(self):
    move_group = self.move_group

    move_group.set_max_velocity_scaling_factor(0.3)
    move_group.set_max_acceleration_scaling_factor(0.3)

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -pi/2
    joint_goal[1] = -pi/2
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  # -------------------------------ROBOT MOTION ----------------------------------------------------  
  def target_motion_save(self, time, plan, vel, acce):
    move_group = self.move_group

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('testos')
    path = pkg_path + '/src/traj_joints'
    path2 = pkg_path + '/src/traj_pose'

    move_group.set_max_velocity_scaling_factor(vel) 
    move_group.set_max_acceleration_scaling_factor(acce)

    move_group.execute(plan, wait=False)

    for x in range (time+2):
      pos = move_group.get_current_joint_values()
      with open (path,'a') as file1:
        for y in range(6):
          if y != 5:             
            file1.write(str (pos[y]) + ",")
          else:
            file1.write(str (pos[y]) + "\n")
            
      pos = move_group.get_current_pose()
      with open (path2,'a') as file2:           
        file2.write(str (pos.pose.position.x) + "," + str (pos.pose.position.y) + "," + str (pos.pose.position.z) + "\n")
      rospy.sleep(0.1)

    file1.close()
    file2.close()

    move_group.stop()
    move_group.clear_pose_targets()
  # -------------------------------------------------------  
  def target_motion(self):
    move_group = self.move_group

    move_group.allow_replanning(True)
    move_group.set_planner_id("RRTstarkConfigDefault")
    move_group.set_planning_time(4)
    move_group.set_num_planning_attempts(100)

    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.position.x = 0.25
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.6

    vel = 0.3
    acce = 0.3

    move_group.set_max_velocity_scaling_factor(vel) 
    move_group.set_max_acceleration_scaling_factor(acce)
    move_group.set_pose_target(pose_goal)

    plan = move_group.plan()

    start = time.time()
    move_group.execute(plan, wait=True)
    end = time.time()

    times = int(round(end-start,1)*10)

    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = self.move_group.get_current_pose().pose
    all_close(pose_goal, current_pose, 0.01)

    return times, plan, vel, acce
  #---------------------------------------------------------------
  def clear(self):
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('testos')
    path = pkg_path + '/src/traj_joints'
    path2 = pkg_path + '/src/traj_pose'

    with open (path,'a') as file1: 
      file1.truncate(0)
    file1.close()

    with open (path2,'a') as file2: 
      file2.truncate(0)
    file2.close()

    return True
  #--------------------------------PLANNING SCENE----------------------------------------------
  def add_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"

    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.05

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(2, 2, 0.1))
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

  def remove_box(self):
    scene = self.scene
    scene.remove_world_object()
#--------------------------------------------------------------------------------------------------
def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "------------- SAVING PATH SCRIPT -------------------------"
    print "----------------------------------------------------------"
    print ""
    print "=========>>> PRESS 'ENTER' CONFIGURATION"

    raw_input()
    tar_sim = Target_simulation() 
    rospy.sleep(2)

    tar_sim.add_box()

    rospy.sleep(0.5)
    tar_sim.init_motion()

    print ("\n[1]=SHOW TRAJECTORY \n[2]=SAVE PLAN TRAJECTORY \n[3]=SIMPLY MOTION TO TARGET \n")
    try:
      num =  int(input("CHOOSE: "))
    except Exception as ex:
      print ("YOU DONT CHOOSE NUMBER!")
      sys.exit()

    @case
    def case1():
      print ("YOU CHOOSE: SHOW TRAJECTORY")
      plot=show.Graph_3D()
      positions=plot.o_traj_pose()
      plot.show_plot(positions)

    @case
    def case2():
      print("YOU CHOOOSE: SAVE TRAJECTORY")
      tar_sim.clear()
      time,plan,velo,accele = tar_sim.target_motion()
      tar_sim.init_motion()
      tar_sim.target_motion_save(time,plan,velo,accele)

    @case
    def case3():
      print("YOU CHOOOSE: SIMPLY MOTION TO GOAL")
      tar_sim.target_motion()

    @case
    def default_case():
      print('YOU CHOOSE NUMBER OF RANGE')

    switch(num, {
      1: case1(),
      2: case2(),
      3: case3(),
    }, default_case())

    tar_sim.remove_box()
    print "=========>>> FINISH!"

  #-------------------------EXCEPTIONS------------------------------
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

