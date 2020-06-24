#!/usr/bin/env python

import sys
import rospy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 

from math import pi
import rospkg
import numpy as np

import show

from NeoPySwitch import SwitchCase as case
from NeoPySwitch import PySwitch as switch

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

class Target_real(object):
  def __init__(self):
  # ---------------- INICIALIZATION ------------------------
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('TESTOS', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    
    self.box_name = ''
    self.scene = scene
    self.robot = robot
    self.move_group = move_group
  # -----------------------------------------------
  def init_motion(self):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -pi/2
    joint_goal[1] = -pi/2
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
      	
    move_group.set_max_velocity_scaling_factor(0.1) #(0;1]
    move_group.set_max_acceleration_scaling_factor(0.1)
      
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
  # -----------------------------------------------
  def compute(self, path, tar):
    new = []
    nex = []

    comp = 100
    b = np.array((tar[0], tar[1], tar[2]))

    with open(path,'r') as f:
      pos = [np.float_(x) for line in f for x in line.rstrip().split(',')]
    f.close()

    for i in range(1,len(pos)):
      if pos[i] < 1:
        new.append(pos[i])

    for i in range(0,len(new),3):
      a = np.array((new[i], new[i+1],new[i+2]))
      distance = np.linalg.norm(a - b)

      if distance < comp:
        comp=distance
        nex = a

    for i in range(len(pos)):
      if nex[0] == pos[i] and nex[1] == pos[i+1] and nex[2] == pos[i+2]:
        for y in range(i,len(pos)):
          if pos[y] > 1:
            info = pos[y]
            break
    
    new =[]

    for i in range(len(pos)):
      if pos[i] == info-1:
        for i in range(i+1,len(pos)):             
          if nex[0] == pos[i] and nex[1] == pos[i+1] and nex[2] == pos[i+2]:
            break
          else:
            new.append(pos[i])

    return info, new
  #----------------------------------------------------------------------------
  def f_distance(self, x, y, z, tar):
    done = False

    a = np.array((x, y, z))               
    b = np.array((tar[0], tar[1], tar[2]))
    distance = np.linalg.norm(a - b)

    print (distance)
    if distance < 0.01:
      done = True
    
    return done
  #-----------------------------------------------------------------------------
  def back_motion_rl(self, info, path, tar):
    move_group = self.move_group

    new = []

    with open(path,'r') as f:
      pos = [np.float_(x) for line in f for x in line.rstrip().split(',')]
    f.close()
    
    for i in range(len(pos)):
      if pos[i] == info-1:
        for i in range(i,len(pos)):             
          if pos[i] == info:
            break
          new.append(pos[i])
    
    move_group.allow_replanning(True)

    move_group.set_max_velocity_scaling_factor(0.1) #(0;1]
    move_group.set_max_acceleration_scaling_factor(0.1)
    
    for i in range(1,len(new),6):
      s = move_group.get_current_pose()
      
      Q = [new[i], new[i+1], new[i+2], new[i+3], new[i+4], new[i+5]]
      move_group.set_joint_value_target(Q) 
      move_group.go(wait=True)

      print (s.pose.position.x,s.pose.position.y,s.pose.position.z)
      prom = self.f_distance(s.pose.position.x,s.pose.position.y,s.pose.position.z,tar)
      
      if prom is True:
        move_group.stop()
        sys.exit()

    move_group.stop()  
  # ------------------------------------------------------------------------------------
  def back_motion(self, path):
    move_group = self.move_group

    with open(path,'r') as f:
      pos = [np.float_(x) for line in f for x in line.rstrip().split(',')]
    f.close()

    move_group.allow_replanning(True)

    move_group.set_max_velocity_scaling_factor(0.1) #(0;1]
    move_group.set_max_acceleration_scaling_factor(0.1)
      
    for i in range(0,len(pos)-1,6):
      Q = [pos[i], pos[i+1], pos[i+2], pos[i+3], pos[i+4], pos[i+5]]
      move_group.set_joint_value_target(Q) 
      move_group.go(wait=False)
      
    move_group.stop()

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
#----------------------------MAIN-----------------------------------------
def main():
  tar = [0.25, 0.0, 0.6]

  rospack = rospkg.RosPack()
  pkg_path = rospack.get_path('testos')

  pathdqn_joints = pkg_path + '/src/traj_dqnjoints'
  pathdqn_pose = pkg_path + '/src/traj_dqnpose'

  pathdsarsa_joints = pkg_path + '/src/traj_dsarsajoints'
  pathdsarsa_pose = pkg_path + '/src/traj_dsarsapose'

  pathq_joints = pkg_path + '/src/traj_qjoints'
  pathq_pose = pkg_path + '/src/traj_qpose'

  paths_joints = pkg_path + '/src/traj_sjoints'
  paths_pose = pkg_path + '/src/traj_spose'

  path_planner = pkg_path + '/src/traj_joints'
  
  name_dqn = 'DQN'
  name_dsarsa = 'DEEP SARSA'
  name_q = 'Q-learning' 
  name_s = 'SARSA'
  name_p = 'MoveIT planner'
  
  try:
    print ""
    print "----------------------------------------------------------"
    print "------------- OPENING PATH SCRIPT ------------------------"
    print "----------------------------------------------------------"
    print ""
    print "=========>>> PRESS 'ENTER' CONFIGURATION"

    raw_input()
    tar_real = Target_real()
    plot=show.Graph_3D()

    rospy.sleep(2)

    tar_real.add_box()

    rospy.sleep(0.5) 
    tar_real.init_motion()
    rospy.sleep(0.5)
    

    print ("\n[1]=SHOW TRAJECTORY Q-LEARNING \n[2]=SHOW TRAJECTORY SARSA \n[3]=SHOW TRAJECTORY DQN \n[4]=SHOW TRAJECTORY DEEP SARSA \n[5]=SHOW TRAJECTORY FROM PLANNING ALGORITHMS\n")
    try:
      num =  int(input("CHOOSE: "))
    except Exception as ex:
      print ("YOU DONT CHOOSE NUMBER!")
      sys.exit()

    @case
    def case1():
      print ("YOU CHOOSE: SHOW TRAJECTORY Q-LEARNING")
      info, positions = tar_real.compute(pathq_pose,tar)
      plot.show_plot(positions,name_q)
      tar_real.back_motion_rl(info,pathq_joints,tar)

    @case
    def case2():
      print("YOU CHOOOSE: SHOW TRAJECTORY SARSA")
      info, positions = tar_real.compute(paths_pose,tar)
      plot.show_plot(positions,name_s)
      tar_real.back_motion_rl(info,paths_joints,tar)
    
    @case
    def case3():
      print("YOU CHOOOSE: SHOW TRAJECTORY DQN")
      info, positions = tar_real.compute(pathdqn_pose,tar)
      plot.show_plot(positions,name_dqn)
      tar_real.back_motion_rl(info,pathdqn_joints,tar)
    
    @case
    def case4():
      print("YOU CHOOOSE: SHOW TRAJECTORY DEEP SARSA")
      info, positions = tar_real.compute(pathdsarsa_pose,tar)
      plot.show_plot(positions,name_dsarsa)
      tar_real.back_motion_rl(info,pathdsarsa_joints,tar)

    @case
    def case5():
      print("YOU CHOOOSE: SHOW TRAJECTORY FROM PLANNING ALGORITHMS")
      tar_real.back_motion(path_planner)
      positions=plot.o_traj_pose()
      plot.show_plot(positions,name_p)
      rospy.sleep(10)

    @case
    def default_case():
      print('YOU CHOOSE NUMBER OF RANGE')

    switch(num, {
      1: case1(),
      2: case2(),
      3: case3(),
      4: case4(),
      5: case5(),
    }, default_case())
    
    tar_real.remove_box()

    print "=========>>> FINISH!"    
  #-------------------------EXCEPTIONS------------------------------
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

