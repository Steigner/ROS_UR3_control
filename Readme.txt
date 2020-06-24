***********************************************************************
*Author: Martin Juricek					              *
*Supervisor: Roman Parak					      *
*Result of work in video: https://uloz.to/file/K2YvFkZov6SP/video-mp4 *
***********************************************************************

* GitHub *

https://github.com/Steigner/UR3_control

abbreviation for better orienation: RL-Reinforcement learning
				    DRL-Deep Reinforcement learning
				    Q=Q-learning
				    S=SARSA
				    DQN=Deep Q-network
				    DS=Deep SARSA
				    s_traj=save_trajectory
				    r_traj=read_trajectories

-------------------------------------------------------------------
I) 		   	Install dependencies	          	  |
-------------------------------------------------------------------

1. Install ROS Kinetic
	Guide on: http://wiki.ros.org/kinetic/Installation/Ubuntu

2. Install ROS dependencies
	Following: sudo apt-get install ros-kinetic-catkin python-catkin-tools

3. Create catkin workspace and download neccesary metapackage:
	Universal Robots guide on: https://github.com/ros-industrial/universal_robot

4. Into workspace integrate packages:
	a) UR_modern_driver: git clone https://github.com/ros-industrial/ur_modern_driver.git
	b) testos: git clone https://github.com/Steigner/UR3_control

-------------------------------------------------------------------
| 	     Control real UR3 by default training data!           |
-------------------------------------------------------------------

1] user@user-pc:~$ roslaunch ur_modern_driver ur3_bringup.launch robot_ip:=ROBOT_IP_ADDRESS
2] user@user-pc:~$ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch limited:=true
3] user@user-pc:~$ rosrun testos r_traj.py

-------------------------------------------------------------------
| 	     Simulation by only with framework Moveit!            |
-------------------------------------------------------------------

1] user@user-pc:~$ roslaunch ur_gazebo ur3.launch limited:=true
2] user@user-pc:~$ roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true limited:=true
3] user@user-pc:~$ rosrun testos s_traj.py

-------------------------------------------------------------------
| 	  Simulation with framework Moveit and Package testos!    |
-------------------------------------------------------------------
	    (In package testos is integrated draught tube)

1] user@user-pc:~$ roslaunch testos rl.launch
2] user@user-pc:~$ roslaunch ur3_moveit_planning.launch
3] user@user-pc:~$ rosrun testos s_traj.py


-------------------------------------------------------------------
II) 	     Install dependencies for traning by RL or DRL        |
-------------------------------------------------------------------

1. Update python in ROS (pip3 is need as well)	
	Following: sudo apt-get install python3-yaml		   
	           sudo pip3 install rospkg catkin_pkg 
		  (sudo pip3 install --user rospkg catkin_pkg)

	If will not still working try: export PYTHONPATH=/opt/ros/kinetic/lib/python2.7/dist-packages/

2. Install Open AI
	Following: sudo pip3 install gym

2. Install Tensorflow, Keras
	Update pip: sudo pip3 install --upgrade pip
		    sudo pip3 install --upgrade setuptools
 	* Tensorflo *
	Following: sudo pip3 install tensorflow
		  (sudo pip3 install tensorflow-gpu)
	* KERAS *	
	Following: sudo pip3 install keras
	
	But from my experiences it will be not such easy.
	More instructions on: https://www.tensorflow.org/install/pip

-------------------------------------------------------------------
| 	     Simulation control by RL and DRL		          |
-------------------------------------------------------------------

1] user@user-pc:~$ roslaunch testos rl.launch
2] user@user-pc:~$ roslaunch ur3_moveit_planning.launch

* Q-learning *
3] user@user-pc:~$ rosrun testos run_Q.py

* SARSA *
3] user@user-pc:~$ rosrun testos run_S.py

* DQN *
3] user@user-pc:~$ rosrun testos run_DQN.py

* DSARSA *
3] user@user-pc:~$ rosrun testos run_DS.py

Training data is saved to folder traning_results for RL algorithms and to traning_results2 for DRL.





