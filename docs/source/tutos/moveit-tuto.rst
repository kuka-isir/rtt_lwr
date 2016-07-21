MoveIt! with rtt_ros_control_embedded
=====================================

As the other packages, we'll use the ``run.launch`` from ``lwr_utils`` that we duplicated in the ``lwr_moveit_config`` package for custom arguments.

.. code::

    roslaunch lwr_moveit_config run.launch sim:=true


Then you can list available ros_control controllers :

.. code::

    rosrun controller_manager controller_manager list


.. image:: /_static/move_it.png

.. note::

    These commands launch the following RTT Components :

    .. code::

        o gazebo
        o lwr_sim
        o rtt_ros_control_embedded
            --> controller_manager
            --> hardware interface

    Then you have access to the full ``ros_control`` interface as a normal ROS module, so you can create you own ros_controllers.
    http://wiki.ros.org/ros_control.
