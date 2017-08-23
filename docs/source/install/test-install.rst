Test your installation
======================

We're gonna test every components to make sure everything is working correctly.

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

.. code::

    gazebo -s libgazebo_ros_paths_plugin.so -s libgazebo_ros_api_plugin.so --verbose

Upload the robot's URDF
~~~~~~~~~~~~~~~~~~~~~~~

.. code::

    roslaunch lwr_description lwr_upload.launch
    # you can also pass load_ati_sensor:=true load_handle:=true load_base:=true

.. note:: If the ROS API is loaded correctly, this command should exit immediately

Spawn to robot into Gazebo
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code::

    roslaunch lwr_utils spawn_robot.launch robot_name:=lwr_sim

.. note:: If the model has been correctly uploaded, this command should also exit immediately

.. image:: /_static/gazebo-lwr.png


Test the rtt_lwr tools
-----------------------

.. warning::

    **Close** all previous nodes, deployers, windows etc, and start the main deployer with gazebo.
    
    Now gazebo is launched **inside** the orocos deployer !

.. code::

    roslaunch lwr_utils run.launch sim:=true

.. note::

    You can see the robot in the gazebi gui because the model is spawned into gazebo via the main launch file.
    Still, it has no interface to send commands, what we'll do in the next step.

Now type :

.. code-block:: ruby

    # Load the lwr interface in the deployer
    loadRobot(getRobotName(), isSim(), true)
    # Let it load, then print the components :
    ls

You should see all the components ``running [R]`` :

.. image:: /_static/test-rtt-lwr.png

.. warning::

    The Gazebo server is launched **inside** the ``gazebo`` component (that you can see if you type ``ls``).
    You **will not** be able to launch gazebo separately, it has to be instanciated inside the orocos deployer via this method.
    This component is provided by the https://github.com/kuka-isir/rtt_gazebo_embedded package.
