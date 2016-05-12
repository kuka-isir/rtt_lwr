OROCOS RTT on Xenomai
=====================

Orocos RTT is fully compatible with Xenomai, but we have to build from source.
The rest of the `installation instructions </install/install.html>`_ remains unchanged.


Remove OROCOS from debians
--------------------------

.. code-block:: bash

    # First remove any occurence of debians rtt
    sudo apt-get remove ros-indigo-rtt* ros-indigo-orocos-toolchain

Compile OROCOS for Xenomai
--------------------------

Download the OROCOS toolchain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    # Create the workspace
    mkdir -p ~/orocos_ws/src
    cd ~/orocos_ws
    # Initialize the workspace
    catkin init
    # We want an install + explicit workspace extend (cleaner that way)
    catkin config --install --extend /opt/ros/indigo
    # Get rtt + rtt_ros
    cd ~/orocos_ws/src
    wstool init
    wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/rtt_lwr_xenomai.rosinstall

Build the toolchain
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    # Set orocos target for xenomai if not already done in your bashrc
    export OROCOS_TARGET=xenomai
    # Build (for distcc add -p$(distcc -j) -j$(distcc -j) --no-jobserver)
    catkin build

    # Load the workspace
    unset ROS_PACKAGE_PATH
    source ~/orocos_ws/install
