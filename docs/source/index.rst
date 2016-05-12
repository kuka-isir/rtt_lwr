.. RTT LWR documentation master file, created by
   sphinx-quickstart on Tue May 10 17:18:03 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to the Kuka LWR's documentation at ISIR
===============================================

.. image:: _static/isir.png
    :width: 100px
    :align: left

.. image:: _static/cnrs.jpg
    :width: 100px

.. image:: _static/upmc.png
    :width: 250px

.. toctree::
    :name: install
    :caption: Installation and Configuration
    :glob:
    :maxdepth: 2

    Installation <install>
    Test your installation <test-install>

.. toctree::
    :name: tutos
    :caption: Tutorials
    :glob:
    :maxdepth: 2

    Getting started with OROCOS <orocos-tuto>
    Getting Started with RTT LWR <getting-started>
    Getting started with ros_control / MoveIt! <moveit-tuto>
    Creating your first controller <controller-tuto>

.. toctree::
    :name: rtpc
    :caption: Control PC setup
    :glob:
    :maxdepth: 2

    Xenomai 2.6.4 on Ubuntu 14.04 <xenomai>
    RTnet setup <rtnet>
