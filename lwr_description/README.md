Kuka LWR 4+ Description
================

This package contains the URDF (with inertia matrices) for the Kuka LWR 4+ at ISIR.

### Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/ahoarau/lwr_description.git
```

### Usage

```bash
# Upload the robot description in the parameter server
roslaunch lwr_description lwr_upload.launch
# Launch the fake robot in rviz (to debug)
roslaunch lwr_description lwr_test.launch
```

> Author: Antoine Hoarau <hoarau.robotics@gmail.com>
