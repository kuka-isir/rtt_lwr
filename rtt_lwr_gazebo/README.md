RTT LWR Gazebo
==================

This package also includes a simple demonstration of the RTT Gazebo plugins
and also uses ROS for launching gazebo.

First, launch gazebo with the demo model:
```shell
roslaunch rtt_lwr_gazebo lwr_gazebo.launch
```

Then, in another shell, you can launch the rtt\_gazebo\_console:
```shell
rosrun rtt_gazebo_console console
```

This will display something similar to this:
```shell
   Switched to : console_deployer

  This console reader allows you to browse and manipulate TaskContexts.
  You can type in an operation, expression, create or change variables.
  (type 'help' for instructions and 'ls' for context info)

    TAB completion and HISTORY is available ('bash' like)

    Use 'Ctrl-D' or type 'quit' to exit this program.

console_deployer [S]> 
```

Once you've loaded up the console, you can `cd` into the demo component:
```shell
cd gazebo
cd sevenbot
```

Then you can list some of the debug attributes:
```
ls debug
```

This will display something simiar to:
```shell
 Listing Service debug[R] :

 Configuration Properties: (none)

 Provided Interface:
  Attributes   : 
     double time_rtt       = 6.938               
     double time_gz        = 6.938               
        int steps_rtt      = 6891                
        int steps_gz       = 6771                
        int n_joints       = 8                   
      array joint_pos      = { [-2.78448e-06, 7.2723e-06, 6.19356e-06, -5.0639e-06, 3.9167e-06, -2.79475e-06, 1.73491e-06, 8.32204e-07 ], size = 8, capacity = 8 }
      array joint_command  = { [3.06293e-06, 8.01095e-06, -6.80253e-06, 5.57588e-06, -4.30829e-06, 3.07115e-06, -1.90659e-06, -9.20536e-07 ], size = 8, capacity = 8 }

  Operations      : (none)

 Data Flow Ports: (none)

 Services: 
(none)
```

You can also change the gains of the joint-level PD effort control:
```
kp = 0.5
kd = 10
```
