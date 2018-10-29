# ROS Publisher and Subscriber Implementation
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
Open terminal and run following command to clone this repository:
```
$ git clone https://github.com/srane96/beginner_tutorials.git catkin_ws
$ cd catkin_ws
```
#### Build the package
Then build the package using following command
```
$ catkin_make
```

#### Run the code
First initiate the ros master
```
$ roscore
```
Open new terminal in the workspace and run following commands to start publisher
```
$ source devel/setup.bash
$ rosrun beginner_tutorials talker
```
Open another terminal in the workspace and run following commands to start subscriber
```
$ source devel/setup.bash
$ rosrun beginner_tutorials listener
```
