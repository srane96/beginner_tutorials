[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# ROS Publisher and Subscriber Implementation with Services
This project implements package called beginner_tutorials that implements two nodes:
1. Publisher node - talker
2. Subscriber node - listener

### Overview
Both publisher and subscriber nodes are written using C++. Publisher node is programmed
to publish a custom string message on the chatter topic and Subscriber node is
programmed to receive and display the string message published by the talker.

### Requirements/Dependencies
To run the given code, Following are the system requirements:
1. Ubuntu 16.04 OS
2. ROS Kinetic

### Installation process
#### 1. ROS Kinetic:
Install ROS Kinetic using following [link](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

#### Build Catkin Workspace
Open terminal and run following command to make workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ cd ~/src
$ git clone -b Week10_HW https://github.com/srane96/beginner_tutorials.git catkin_ws
$ cd ..
```
#### Build the package
Then inside work directory build the package using following command
```
$ catkin_make
```

#### Run the code
First initiate the ros master
```
$ roscore
```
Open new terminal in the workspace and run following commands to start the launch file
```
$ source devel/setup.bash
$ roslaunch beginner_tutorials service.launch rate:=<frequency value in int>
```
Open another terminal in the workspace and run following commands to start the service
```
$ source devel/setup.bash
$ rosservice call manipulate_service <integer>
```
