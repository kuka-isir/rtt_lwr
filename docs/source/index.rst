
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
It relies on OROCOS for the real-time part, but also interfaces with ROS such as Rviz, MoveIt, ros-control etc.

It has been designed so researchers/Phd Students/Engineers at ISIR can develop generic controllers for light weight robots and seemlessly test on simulation of the real robot without recompiling their code.

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
