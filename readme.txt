	University of Porto 
	Faculty of Engineering
	Robotics

Assignment 2: Reactive wall following robot.

Author: Samuel Arleo Rodriguez up201600802

Program information:
* Running on Ubuntu Xenial

* ROS version: Kinetic

* Python 2.7

* Package name: myrobot_gazebo

* Package structure:

	- Normal catkin working directory structure (src, devel build)
	- ./catkin/src/ contains main CMakeLists.txt file and package folder myrobot_gazebo
	- ./catkin/src/myrobot_gazebo contains folders:
		* launch: Two .launch files for omap and umap world
		* worlds: Two .world files for Omap and Umap
		* Scripts: robot.py file with node code

* Nodes: robot.py

* Simulator used: Turtlebot Gazebo

* Robot data visualizer: Rviz

Instructions:

0- Run 
	$ source /source catkin/devel/setup.bash
	$ source /source /opt/ros/kinetic/setup.bash 

1- Open 3 terminals and run each command in a different term:
	$ roslaunch myrobot_gazebo omap.launch # or umap.launch
	$ roslaunch turtlebot_rviz_launchers view_robot.launch
	$ rosrun myrobot_gazebo robot.py

3- To move the robot from its initial position use gazebo simulator

Note: With current walls and position of robot radar can make it crash when surrounding a corner.
