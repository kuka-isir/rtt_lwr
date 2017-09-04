
OROCOS/ROS Components for Light Weight Robots
==============================================

.. image:: /_static/isir.png
    :width: 100px
    :align: left

.. image:: /_static/cnrs.jpg
    :width: 100px

.. image:: /_static/upmc.png
    :width: 250px

Introduction
------------

**rtt_lwr** is a set of components for controlling the Kuka LWR and IIWA at 1Khz.
It relies on OROCOS for the real-time part, but also interfaces with ROS so we can use Rviz, MoveIt, ros-control etc.

It has been designed so researchers/Phd Students/Engineers at ISIR can develop generic controllers for light weight robots and seemlessly switch between simulation/real hardware without the need to recompile their code.

Prerequisites
-------------

* Relatively powerful Ubuntu 16.04/14.04 PC
* Knowledge about ROS
* Notions about OROCOS

Design
------

* The robot is kuka lwr 4+ with FRI with KRC4 controller that requires **strict** loop deadlines if we want to achieve 1kHz
* We want to control the robot as fast as possible because we want to test control theory algorithms.
* **Syroco's** team work at **isir** is focused on control theory.

OROCOS and ros_control seems to be the only two options that give us a lot of flexibily, a high control rate, and the ability to run in a realtime environnement.

We choose OROCOS because we wanted to design generic "black boxes", that we could reuse as much as possible.
ros_control gives the ability to launch multiple controllers, the intra-process communication is (was ?) not supported.

========================  =============  ===============  ==========================  ==============
                          Realtime safe  Component based  Intraprocess communication  Well supported
========================  =============  ===============  ==========================  ==============
**FRI library (client)**   X
**ros_control**            X                X                                           X
**OROCOS**                 X                X               X                           X
========================  =============  ===============  ==========================  ==============


Final design is the following :

* One deployer on the realtime control PC (running xenomai)
* Control components inside that only deployer
* ROS bridge for non-realtime components



Experimental setup
------------------

.. image:: https://docs.google.com/drawings/d/1E0KbzYNJTc-1nIdF8U4vIk07o5m0t-UEVZyddy6xKDc/pub?w=1802&amp;h=1195


.. toctree::
    :name: install
    :caption: Installation and Configuration
    :glob:
    :maxdepth: 2

    install/*

.. toctree::
    :name: tutos
    :caption: Tutorials
    :glob:
    :maxdepth: 2

    Getting started with OROCOS <tutos/orocos-tuto>
    Getting started with RTT LWR <tutos/getting-started>
    Getting started with ros_control / MoveIt! <tutos/moveit-tuto>
    tutos/controller-tuto
    tutos/kdev-tuto
    tutos/update-pkgs
    tutos/build-docs

.. toctree::
    :name: tools
    :caption: Tools
    :glob:
    :maxdepth: 2

    tools/*

.. toctree::
    :name: adv-tutos
    :caption: Advanced Tutorials
    :glob:
    :maxdepth: 2

    adv-tutos/*

.. toctree::
    :name: rtpc
    :caption: Control PC setup
    :glob:
    :maxdepth: 2

    rtpc/xenomai
    rtpc/rtnet
    rtpc/orocos-xenomai
