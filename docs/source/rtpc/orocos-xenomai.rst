OROCOS RTT on Xenomai
=====================

Orocos RTT is fully compatible with Xenomai, but it needs to be built *from source*.

The only difference in the build process is the need to set ``export OROCOS_TARGET=xenomai`` before compiling.

Orocos Toolchain 2.9 on Xenomai
-------------------------------

.. code-block:: bash

    # Compile for Xenomai
    export OROCOS_TARGET=xenomai

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

    cd ~/isir/orocos-2.9_ws/
    # Install dependencies
    source /opt/ros/kinetic/setup.bash
    rosdep install --from-paths ~/isir/orocos-2.9_ws/src --ignore-src --rosdistro kinetic -y -r
    # Configure the build in Release (recommended), and enable extra transport methods (optional)
    catkin config --init --install --extend /opt/ros/kinetic/ --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_MQ=ON -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB
    # Build
    catkin build

RTT ROS Integration 2.9 on Xenomai
----------------------------------

.. code-block:: bash

    # Compile for Xenomai
    export OROCOS_TARGET=xenomai

    mkdir -p ~/isir/rtt_ros-2.9_ws/src
    cd ~/isir/rtt_ros-2.9_ws/src
    # Get all the packages
    wstool init
    wstool merge https://github.com/kuka-isir/rtt_lwr/raw/rtt_lwr-2.0/lwr_utils/config/rtt_ros_integration-2.9.rosinstall
    wstool update -j2

    cd ~/isir/rtt_ros-2.9_ws/
    # Install dependencies
    source ~/isir/orocos-2.9_ws/install/setup.bash
    rosdep install --from-paths ~/isir/rtt_ros-2.9_ws/src --ignore-src --rosdistro kinetic -y -r
    # Configure the build in Release (recommended), and enable extra transport methods (optional)
    catkin config --init --install --extend ~/isir/orocos-2.9_ws/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_MQ=ON -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB
    # Build (this can take a while)
    catkin build

.. tip::

    Always build for Xenomai on the Xenomai computer :
    ``echo 'export OROCOS_TARGET=xenomai' >> ~/.bashrc``

.. note::

    Libraries built with xenomai will have *-xenomai.so* appended to the library name (ex ``libmycontroller-xenomai.so``)
