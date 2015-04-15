RTT LWR Abstract
============

This package contains an abstract class which can be derived to create Orocos component
that can communicate with the component `lwr_fri`. The abstract class contains
the basic method to interact with the krl scripts that are in the `script` directory.
This package also contains basic examples, demontrating the use of the abstract class.

Install:
========

In your ros workspace directory:

    rosws set -y fri_examples --git https://github.com/XDE-ISIR/fri_examples.git
    rosws update fri_examples
    cd fri_examples
    rosmake

Copy the krl scripts located in `script` to the KRC computer and reboot it.

Tests example:
==============

There are three examples located in the `tests` folder:
 - FriExample : How to read data from fri
 - FriExampleKinematic : How to send velocity commands to the robot
 - FriExampleTorque : How to send torque commands to the robot
Each examples comes with an Orocos script to test the component.

If you have compiled the orocos toolchain for xenomai you can do:

    cd tests/orocos
    rosrun ocl deployer-xenomai -s friExampleTorque.ops
