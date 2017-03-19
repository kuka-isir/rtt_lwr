#/usr/bin/env sh

STARTPOINT=$(pwd)

mkdir -p ~/orocos-2.9_ws/src
cd ~/orocos-2.9_ws/src
wstool init
wstool merge https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/config/orocos_toolchain-2.9.rosinstall
wstool update -j2

# Get the latest updates

cd orocos_toolchain
git submodule foreach git checkout toolchain-2.9
git submodule foreach git pull

# Configure the workspace
cd ~/orocos-2.9_ws/
catkin init
catkin config --install --extend /opt/ros/$ROS_DISTRO
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build
catkin build

mkdir -p ~/rtt_ros-2.9_ws/src
cd ~/rtt_ros-2.9_ws/src
wstool init
wstool merge https://github.com/kuka-isir/rtt_lwr/raw/rtt_lwr-2.0/lwr_utils/config/rtt_ros-2.9.rosinstall
wstool update -j2

# Configure the workspace
cd ~/rtt_ros-2.9_ws/
catkin init
catkin config --install --extend ~/orocos-2.9_ws/install
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build (this can take a while)
catkin build

source ~/rtt_ros-2.9_ws/install/setup.bash

cd $STARTPOINT
