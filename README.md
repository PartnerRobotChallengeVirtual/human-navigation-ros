# ROS Package for Human Navigation

This project is ROS package for the Human Navigation task of the Partner Robot Challenge (Virtual Space) in the World Robot Competition.

## Prerequisites

- OS: Ubuntu 16.04
- ROS distribution: Kinetic Kame

## How to Install

### Install Rosbridge Server

Please see below.  
http://wiki.ros.org/rosbridge_suite

### Install SIGVerse Rosbridge Server

Please see below.  
https://github.com/SIGVerse/ros_package/tree/master/sigverse_ros_bridge

### Install ROS Package of Interactive Cleanup

```bash:
$ cd ~/catkin_ws/src
$ git clone https://github.com/PartnerRobotChallengeVirtual/human-navigation-ros.git
$ cd ..
$ catkin_make
```

## How to Execute

### How to Execute Sample ROS Node

This sample ROS node communicates with the Unity application of Human Navigation.  
HSR can be operated with keyboard operation.

```bash:
$ roslaunch human_navigation sample.launch
```

## License

This project is licensed under the SIGVerse License - see the LICENSE.txt file for details.
