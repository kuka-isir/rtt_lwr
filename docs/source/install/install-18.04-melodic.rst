Installation on Ubuntu 18.04
============================

ROS Melodic ++
--------------

From  http://wiki.ros.org/melodic/Installation/Ubuntu

Required tools
~~~~~~~~~~~~~~

.. code-block:: bash

    sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros-latest.list"
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt update
    sudo apt install python-rosdep python-catkin-tools ros-melodic-catkin python-wstool python-vcstool

Fix Locales
~~~~~~~~~~~

.. code-block:: bash

   sudo locale-gen en_US #warnings might occur
   sudo locale-gen en_US.UTF-8
   sudo nano /etc/environment
   # put theses lines
   LANGUAGE=en_US
   LC_ALL=en_US
   # Reboot !

If you type ``perl`` you should not see any warnings.

ROS Melodic Desktop Full
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo apt install ros-melodic-desktop-full

After Install
~~~~~~~~~~~~~

.. code-block:: bash

    # Load The environment
    source /opt/ros/melodic/setup.bash
    # Update ROSdep (to get dependencies automatically)
    sudo rosdep init
    rosdep update


MoveIt! (via debians)
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    # MoveIt!
    sudo apt install ros-melodic-moveit

OROCOS 2.9 + rtt_ros_integration 2.9 (from source)
--------------------------------------------------

OROCOS toolchain 2.9
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    mkdir ~/isir ; cd ~/isir
    mkdir -p orocos-2.9_ws/src
    cd orocos-2.9_ws/src
    # Get all the packages
    wstool init
    wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/orocos_toolchain-2.9.rosinstall
    wstool update -j2
    # Get the latest updates (OPTIONAL)
    cd orocos_toolchain
    git submodule foreach git checkout toolchain-2.9
    git submodule foreach git pull
    # Configure the workspace
    cd ../../
    # Install dependencies
    source /opt/ros/melodic/setup.bash
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r
    catkin config --init --install --extend /opt/ros/melodic/ --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Build
    catkin build

rtt_ros_integration 2.9
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    cd ~/isir
    mkdir -p rtt_ros-2.9_ws/src
    cd rtt_ros-2.9_ws/src
    # Get all the packages
    wstool init
    wstool merge https://github.com/kuka-isir/rtt_lwr/raw/rtt_lwr-2.0/lwr_utils/config/rtt_ros_integration-2.9.rosinstall
    wstool update -j2
    # Configure the workspace
    cd ../
    # Install dependencies
    source ~/isir/orocos-2.9_ws/install/setup.bash
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r
    catkin config --init --install --extend ~/isir/orocos-2.9_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Build (this can take a while)
    catkin build

ROS Control
-----------

This allows you to use MoveIt! or just the ros_control capabilities in an orocos environnement. Let's install everything :

.. code-block:: bash

    sudo apt install ros-melodic-ros-control* ros-melodic-control*

RTT LWR packages
----------------

.. code-block:: bash

    mkdir -p ~/isir/lwr_ws/src/
    cd ~/isir/lwr_ws/src
    # Get all the packages
    wstool init
    # Get rtt_lwr 'base'
    wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/rtt_lwr.rosinstall
    # Get the extra packages
    wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/rtt_lwr-extras.rosinstall

    # Download
    wstool update -j2

Cart Opt Ctrl
~~~~~~~~~~~~~

Experimental optimisation based controller :

.. code-block:: bash

    wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/rtt_lwr-full.rosinstall
    wstool update

Install dependencies
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    # If you compiled rtt_ros from sources
    source ~/isir/rtt_ros-2.9_ws/install/setup.bash
    # Use rosdep tool
    rosdep install --from-paths ~/isir/lwr_ws/src --ignore-src --rosdistro melodic -y -r

Configure the workspace
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    cd ~/isir/lwr_ws
    catkin config --init --install --extend ~/isir/rtt_ros-2.9_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release

Build the workspace
~~~~~~~~~~~~~~~~~~~

Let's build the entire workspace :

.. code-block:: bash

    catkin build --workspace ~/isir/lwr_ws

.. image:: /_static/catkin-build.png

Once it's done, load the workspace :

.. code-block:: bash

    source ~/isir/lwr_ws/install/setup.bash

.. tip:: Put it in you bashrc : ``echo 'source ~/isir/lwr_ws/install/setup.bash' >> ~/.bashrc``

Now we can :doc:`test the installation <test-install>`.
