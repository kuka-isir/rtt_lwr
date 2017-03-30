Installation instructions
=========================

Requirements
------------

- **Ubuntu 14.04/16.04 LTS**
- **ROS Indigo/Kinetic** - Desktop (not desktop-full)  : http://wiki.ros.org/indigo/Installation/Ubuntu
- **OROCOS 2.8/2.9** ``(from the ros debian repos if not on xenomai)``
- **Gazebo 7** ``(you can work with older version, but please prefer the latest)``

.. note::
    * This installation is valid for ROS Kinetic

ROS Indigo ++
-------------

From  http://wiki.ros.org/indigo/Installation/Ubuntu.

Required tools
~~~~~~~~~~~~~~

.. code-block:: bash

    sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main' > /etc/apt/sources.list.d/ros-latest.list"
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt update
    sudo apt install python-rosdep python-catkin-tools ros-indigo-catkin python-wstool python-vcstool

Fix Locales
~~~~~~~~~~~~~~

.. code-block:: bash
   
   sudo locale-gen en_US #warnings might occur
   sudo locale-gen en_US-UTF-8
   sudo nano /etc/environment
   # put theses lines
   LANGUAGE=en_US
   LC_ALL=en_US
   # Reboot !
   
If you type ``perl`` you should not see any warnings.

ROS Indigo Desktop
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    # ROS Desktop (NOT DESKTOP-FULL)
    sudo apt install ros-indigo-desktop

.. warning:: Do not install **desktop-full** (desktop + gazebo 2.2) as we'll use Gazebo 7.

After Install
~~~~~~~~~~~~~

.. code-block:: bash

    # Load The environment
    source /opt/ros/indigo/setup.bash
    # Update ROSdep (to get dependencies automatically)
    sudo rosdep init
    rosdep update


MoveIt! (via debians)
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    # MoveIt!
    sudo apt install ros-indigo-moveit-full

MoveIt! (from source)
~~~~~~~~~~~~~~~~~~~~~

If you need bleeing-edge features, compile MoveIt! from source :

.. code-block:: bash

    mkdir -p ~/isir/moveit_ws/src
    cd ~/isir/moveit_ws/src
    # Get all the packages
    wstool init
    wstool merge https://raw.githubusercontent.com/ros-planning/moveit_docs/indigo-devel/moveit.rosinstall
    wstool update -j2
    cd ~/isir/moveit_ws/
    # Install dependencies
    source /opt/ros/indigo/setup.bash
    rosdep install -q --from-paths ~/isir/moveit_ws/src --ignore-src --rosdistro indigo -y -r
    # Configure the workspace
    catkin config --init --install --extend /opt/ros/indigo --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Build
    catkin build

OROCOS 2.8 + rtt_ros_integration 2.8 (via debians)
----------------------------------------------

OROCOS toolchain 2.8
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo apt install ros-indigo-orocos-toolchain ruby1.9.3 ruby-dev libreadline-dev

rtt_ros_integration 2.8
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo apt install ros-indigo-rtt-* ros-indigo-eigen-typekit ros-indigo-kdl-typekit

OROCOS 2.9 + rtt_ros_integration 2.9 (from source)
----------------------------------------------

If you already completed these instructions, and you are upgrading from orocos 2.8 :

- If you installed orocos 2.8 from the debians, you need to remove them ``sudo apt remote ros-indigo-orocos-toolchain ros-indigo-rtt-*``.
- If you installed orocos 2.8 from source, they can live side by side in a **different** workspace, but always check ``catkin config`` on your lwr_ws to make sure which workspace you are extending.

Additionally, please make sure that these repos (if you have them) are in the right branches (with fixes for rtt) :

.. code-block:: bash

    roscd rtt_dot_service && git remote set-url origin https://github.com/kuka-isir/rtt_dot_service.git && git pull
    roscd fbsched && git remote set-url origin https://github.com/kuka-isir/fbsched.git && git pull
    roscd conman && git remote set-url origin https://github.com/kuka-isir/conman.git && git pull

OROCOS toolchain 2.9
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    mkdir -p ~/isir/orocos-2.9_ws/src
    cd ~/isir/orocos-2.9_ws/src
    # Get all the packages
    wstool init
    wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/orocos_toolchain-2.9.rosinstall
    wstool update -j2
    # Get the latest updates (OPTIONAL)
    cd orocos_toolchain
    git submodule foreach git checkout toolchain-2.9
    git submodule foreach git pull
    # Configure the workspace
    cd ~/isir/orocos-2.9_ws/
    # Install dependencies
    source /opt/ros/indigo/setup.bash
    rosdep install -q --from-paths ~/isir/orocos-2.9_ws/src --ignore-src --rosdistro indigo -y -r
    catkin config --init --install --extend /opt/ros/indigo/ --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Build
    catkin build

rtt_ros_integration 2.9
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    mkdir -p ~/isir/rtt_ros-2.9_ws/src
    cd ~/isir/rtt_ros-2.9_ws/src
    # Get all the packages
    wstool init
    wstool merge https://github.com/kuka-isir/rtt_lwr/raw/rtt_lwr-2.0/lwr_utils/config/rtt_ros_integration-2.9.rosinstall
    wstool update -j2
    # Configure the workspace
    cd ~/isir/rtt_ros-2.9_ws/
    # Install dependencies
    source /opt/ros/indigo/setup.bash
    rosdep install -q --from-paths ~/isir/rtt_ros-2.9_ws/src --ignore-src --rosdistro indigo -y -r
    catkin config --init --install --extend ~/isir/orocos-2.9_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release
    # Build (this can take a while)
    catkin build

Use OROCOS with CORBA + MQUEUE (Advanced)
---------------------

In order to use the corba interface (connect multiple deployers together), you'll need to build the orocos_ws and rtt_ros_ws with :

.. code-block:: bash

    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_MQ=ON -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB

Reference : http://www.orocos.org/stable/documentation/rtt/v2.x/doc-xml/orocos-components-manual.html#orocos-corba

Gazebo 7
--------

From http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install.

.. note:: If you already have gazebo 2.2 installed, please remove it : `sudo apt remove gazebo libgazebo-dev ros-indigo-gazebo-*`

.. code-block:: bash

    # Gazebo 7
    curl -ssL http://get.gazebosim.org | sh
    # The ros packages
    sudo apt install ros-indigo-gazebo7-*

.. note:: Don't forget to put source ``source /usr/share/gazebo/setup.sh`` in your ``~/isir/.bashrc`` or you won't have access to the gazebo plugins (Simulated cameras, lasers, etc).

ROS Control
-----------

This allows you to use MoveIt! or just the ros_control capabilities in an orocos environnement. Let's install everything : 

.. code-block:: bash

    sudo apt install ros-indigo-ros-control* ros-indigo-control*

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

.. note:: If you want to install and test cart_opt_ctrl :  ``wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/rtt_lwr-full.rosinstall``

Install dependencies
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    source /opt/ros/indigo/setup.bash
    # Use rosdep tool
    rosdep install -q --from-paths ~/isir/lwr_ws/src --ignore-src --rosdistro indigo -y -r

Configure the workspace
~~~~~~~~~~~~~~~~~~~~~~~

If using the **debians** :

.. code-block:: bash

    cd ~/isir/lwr_ws
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=Release

If building rtt_ros **from source** :

.. code-block:: bash

    cd ~/isir/lwr_ws
    catkin config --init --extend ~/isir/rtt_ros-2.9_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release 

Build the workspace
~~~~~~~~~~~~~~~~~~~

Let's build the entire workspace :

.. code-block:: bash

    catkin build --worspace ~/isir/lwr_ws

.. image:: /_static/catkin-build.png

Once it's done, load the workspace :

.. code-block:: bash

    source ~/isir/lwr_ws/devel/setup.bash

.. tip:: Put it in you bashrc : ``echo `source ~/isir/lwr_ws/devel/setup.bash` >> ~/.bashrc``

Now we can :doc:`test the installation <test-install>`.
