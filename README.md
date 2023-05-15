# High Scale Cartesian Robot for automatic irrigation using URDF and ROS

## Overview
This project involves the design and implementation of a high-scale cartesian robot using the Robot Operating System (ROS) and the Unified Robot Description Format (URDF) file. The robot has a size of 30x10x10 meters and is suitable for industrial applications that require high accuracy and precision.

The robot's mechanical structure is defined in the URDF file, which includes the joint types, dimensions, and limits. The URDF file also defines the robot's visual and collision models, which are used for simulation and collision detection. The robot's kinematics and dynamics are modeled using the URDF-based robot description.The robot's control system is implemented using ROS, which provides a framework for developing and integrating software components.


## Quick Start Guide

### Requirements
For simulating the system, the following elements are required:
* Ubuntu 20.04.06 LTS (Focal Fossa): https://releases.ubuntu.com/focal/
* ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
* RViz for Noetic: http://wiki.ros.org/rviz/UserGuide
* Xacro: http://wiki.ros.org/xacro

### Setup
Download or clone contents of the ROS Folder to your designated Catkin Workspace source folder, in this example the folder is located at:

	cd home/user/catkin_ws/src

Inside the cartesian_irrigator folder, the following folder structure should be included:
![image](https://github.com/Githubense/cartesian_irrigator/assets/77215295/2307b8d7-9fe0-49d6-9e1d-e1f993eb2dbd)

Once downloaded execute the following command:
	
	catkin_make
	
After that, run the setup.bash for the src:

	source devel/setup.bash

### Operation

Open up a terminal and start ROS using this command:

	roscore
	
Once it has mounted thei instance, open up a new terminal and run the launch command, this file contains all the neccesary components:

	roslaunch cartesian_irrigator display.launch
	
The previous command will launch a RViz instance with the URDF robot displayed, such as the following figure describes:
![image](https://github.com/Githubense/cartesian_irrigator/assets/77215295/09ac6b54-975a-42be-9a42-fef443cde7d1)

In order to move the robot in a simulation environment, an instance of the joint_state_pusblisher GUI window is needed, to run it use this command in a new terminal, the following figure will appear afterwards:

	rosrun joint_state_publisher_gui joint_state_publisher_gui
	
![image](https://github.com/Githubense/cartesian_irrigator/assets/77215295/a2f8159f-fb90-464d-8731-8bbaf84d8c4b)

With the included sliders, you can move the robot in the X, Y and Z axis, with the available constraints and limits made by the URDF file itself, the following figures demonstrate the simulation with the sliders.

X Movement:
![x_mov](https://github.com/Githubense/cartesian_irrigator/assets/77215295/ab4968ab-da7f-49e6-9cc0-71caccd33df1)
![x_mov_joint](https://github.com/Githubense/cartesian_irrigator/assets/77215295/8f0e9893-0084-48aa-bb84-5bd70d5f238f)

Y Movement:
![y_mov_2](https://github.com/Githubense/cartesian_irrigator/assets/77215295/31dd98c6-042e-403b-af6f-4d5b5e934b9c)
![y_mov_joint](https://github.com/Githubense/cartesian_irrigator/assets/77215295/1ceec3da-cfb8-4f1f-924c-dd835f2fcbd0)

Z Movement:
![z_mov](https://github.com/Githubense/cartesian_irrigator/assets/77215295/40dad505-3411-4594-a543-462f0089efb4)
![z_mov_joint](https://github.com/Githubense/cartesian_irrigator/assets/77215295/9199010d-630d-492d-8006-8023c0ead137)

## Project Design

### Robot Design
The design of the high-scale cartesian robot for automatic irrigation with dimensions of 30x10x10m was intended to provide a solution that facilitates the use of robotics in agriculture and botanics. With the increasing need for automation in the agriculture industry, the robot's design was tailored to provide a scalable solution that could be used in different applications.

The robot's size is a significant advantage, as it can cover a large area and perform its functions without requiring significant manual intervention. The robot's ability to navigate through fields and cover large distances makes it ideal for use in a variety of crop management applications. The robot's design was also intended to address the challenges associated with traditional irrigation methods. With the color recognition system, the robot can identify plants that need irrigation, reducing the need for water wastage and optimizing plant development. This functionality significantly reduces the cost of labor associated with manual irrigation and improves overall plant health.

### Matlab Implementation

### Connection

## References
