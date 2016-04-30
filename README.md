[![Build Status](https://travis-ci.org/kuka-isir/rtt_lwr.svg?branch=rtt_lwr-2.0)](https://travis-ci.org/kuka-isir/rtt_lwr)

RTT Kuka LWR 4+
===================
- **lwr_description** : contains the URDF, with tools, and launch files associated.
- **lwr_hardware** : fork of [RCPRG's repo](https://github.com/RCPRG-ros-pkg/lwr_hardware). Contains an RTT component for communicating with the hardware (through rtnet) et another for publishing the robot status in ROS.
- **lwr_ikfast** : Fast inverse kinematics plugin for moveit.
- **lwr_moveit_config** : [MoveIt!](http://moveit.ros.org/) config package for starting moveit and connecting it to the controller manager.
- **lwr_utils** : Set of **very** usefull tools to launch everything. 
- **rtt_lwr_abstract** : contains a soon-to-be-simplified-to-be-robot-agnostic class for easy connection with the robot hardware or sim. A lot of functions should be removed to enable a more generic interface.

### Control Software Architecture

[![Kuka LWR 4+ Control Software Architecture 2.0](https://docs.google.com/drawings/d/1CGQaes89flOIwtlaBMlV-LNl8od_qBvY_emp2re9bnE/pub?w=2283&amp)](https://goo.gl/jb8QS9)

## Installation

Please follow [these installation instructions](https://github.com/kuka-isir/rtt_lwr/wiki/Installation). It will install ROS Indigo, Moveit, Gazebo7, and OROCOS

