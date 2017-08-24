Test your installation
======================

We're gonna test a few components to make sure everything is working correctly.

* The ``catkin build`` during install command has no errors
* Gazebo starts normally
* lwr_utils starts normally
* Gazebo inside orocos (embedded) starts normally

.. important::

    The Gazebo server in the final setup is launched **inside** the ``gazebo`` component (that you can see if you type ``ls``).
    You **will not** be able to launch gazebo separately, it has to be instanciated inside the orocos deployer via this method.
    This component is provided by the `rtt_gazebo_embedded <https://github.com/kuka-isir/rtt_gazebo_embedded>`_ package.

Making sure everything is built
-------------------------------

Every one of these commands should provide no errors (all green).

.. code-block:: bash

    # Orocos Toolchain 2.9
    catkin build -s -w ~/isir/orocos-2.9_ws
    # Orocos-ROS bridge
    catkin build -s -w ~/isir/rtt_ros-2.9_ws
    # Rtt lwr
    catkin build -s -w ~/isir/lwr_ws


Gazebo
------
Gazebo needs to get some models at the first launch, so in a terminal type :

.. code::

    gzserver --verbose

.. image:: /_static/gzserver.png

Then ``ctrl+C`` to close it and type :

.. code::

    gazebo

.. image:: /_static/gazebo.png


Gazebo-ROS
----------

Gazebo with ROS plugin
~~~~~~~~~~~~~~~~~~~~~~

Start the roscore : ``roscore``

Close any instance of gazebo running, and then launch gazebo with the ros plugins :

.. code-block:: ruby

    gazebo -s libgazebo_ros_paths_plugin.so -s libgazebo_ros_api_plugin.so --verbose

Upload the robot's URDF
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: ruby

    roslaunch lwr_description lwr_upload.launch
    # you can also pass load_ati_sensor:=true load_handle:=true load_base:=true

.. note:: If the ROS API is loaded correctly, this command should exit immediately

Spawn to robot into Gazebo
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code::

    roslaunch lwr_utils spawn_robot.launch robot_name:=lwr_sim

.. note:: If the model has been correctly uploaded, this command should also exit immediately

.. image:: /_static/gazebo-lwr.png


Gazebo inside the OROCOS Deployer
---------------------------------

.. important::

    **Close** all previous nodes, roscore, deployers, windows etc, and start the main deployer with gazebo.

    Now gazebo is launched **inside** the orocos deployer !

.. code-block:: ruby

    roslaunch lwr_utils run.launch sim:=true
    # this launches gazebo inside the orocos deployer

.. note::

    You can see the robot in the gazebo gui because the model is spawned into gazebo via the main launch file.
    Still, it has no interface to send commands, what we'll do in the next step.

Now type :

.. code-block:: ruby

    # Load the lwr interface in the deployer
    loadRobot(getRobotName(), isSim(), true)
    # Let it load, then print the components :
    ls

You should see all the components ``running [R]`` :

.. image:: /_static/test-rtt-lwr.png
