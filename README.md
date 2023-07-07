![alt text](https://raw.githubusercontent.com/OnOrbitROS/Simulation/main/logo.png)

# Description

This is my final project for the Robotics Engineering degree at the University of Alicante. For this project I used the TALOS robot from the company [PAL Robotics](https://pal-robotics.com/). It is modified and the legs have been removed, so they are not required for the purpose of this project.

As for the implemented controllers, these can be found in the `talos_control` folder. A kinematic and a dynamic controller are provided, both have a version that uses computer vision and one that does not. For those that use vision, it is because an ArUco marker is used for the location of the robot in the environment.


# Installation
Clone this repository into a folder
```bash
git clone https://github.com/OnOrbitROS/robot_torso.git space_ws
cd space_ws
```
Once cloned compile with the following command
```bash
catkin_make
source devel/setup.bash
```
# Usage
To start using the project you must first start the simulation with the following command
```bash
roslaunch talos_gazebo talos_iss.launch
```
This will start gazebo with models of the robot and the International Space Station (ISS). In addition, it will start the nodes needed to create the simulation scenario. 

On the other hand, to start the controllers and be able to test them, it is necessary to open another terminal and execute one of the following commands depending on which controller you wish to test 

|                **Controllers**               	|  **Type** 	| **Use camera** 	|
|:--------------------------------------------:	|:---------:	|:--------------:	|
| aruco_trajectory_cartesian_controller.launch 	| Kinematic 	|       Yes      	|
|       cartesian_controller_basic.launch      	| Kinematic 	|       No       	|
|        torso_effort_controller.launch        	|  Dynamic  	|       Yes      	|
|   cartesian_controller_torso_efforts.launch  	| Kinematic 	|       No       	|

For example to start the `torso_effort_controller` it would be executed as follows
```bash
roslaunch talos_control torso_effort_controller.launch  
```

For Cartesian controllers, a topic is enabled so that the target Cartesian coordinates are sent there.

# Contact
*Author: [@Pabloo05](https://github.com/Pabloo05)* 