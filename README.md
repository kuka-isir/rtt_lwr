RTT Kuka LWR 4+
===================
[![Build Status](https://travis-ci.org/kuka-isir/rtt_lwr.svg?branch=master)](https://travis-ci.org/kuka-isir/rtt_lwr)

## LWR Control Modes

### Joint Impedance Control
<img src="http://www.sciweavers.org/tex2img.php?eq=%5Ctau_%7Bcmd%7D%20%3D%20k_p%28q_%7BFRI%7D%20-%20q_%7Bmsr%7D%29%20-%20k_d.%5Cdot%7Bq%7D%20%2B%20%5Ctau_%7BFRI%7D%20%2B%20%20f_%7Bdynamics%7D%28q%2C%5Cdot%7Bq%7D%2C%5Cddot%7Bq%7D%29&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\tau_{cmd} = k_p(q_{FRI} - q_{msr}) - k_d.\dot{q} + \tau_{FRI} +  f_{dynamics}(q,\dot{q},\ddot{q})" width="590" height="29" />

### Cartesian Impedance Control
<img src="http://www.sciweavers.org/tex2img.php?eq=%5Ctau_%7Bcmd%7D%20%3D%20J%5ET%20%5Bk_c%28x_%7BFRI%7D%20-%20x_%7Bmsr%7D%29%20%20%2B%20F_%7BFRI%7D%5D%20-%20k_d.%5Cdot%7Bq%7D%20%2B%20f_%7Bdynamics%7D%28q%2C%5Cdot%7Bq%7D%2C%5Cddot%7Bq%7D%29&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\tau_{cmd} = J^T [k_c(x_{FRI} - x_{msr})  + F_{FRI}] - k_d.\dot{q} + f_{dynamics}(q,\dot{q},\ddot{q})" width="642" height="31" />

### Control Software Architecture

[![Kuka LWR 4+ Control Software Architecture 2.0](https://docs.google.com/drawings/d/1CGQaes89flOIwtlaBMlV-LNl8od_qBvY_emp2re9bnE/pub?w=2283&amp)](https://goo.gl/jb8QS9)

## Installation

Please follow [this installation script](https://github.com/kuka-isir/lwr_setup). It will install ROS Indigo, Moveit, Gazebo6, and OROCOS (with custom patches while waiting for PRs)

### Test your Installation

First simple test, type ```deployer-corba``` in the terminal :
```
Real-time memory: 517904 bytes free of 524288 allocated.
0.045 [ Warning][Thread] Lowering scheduler type to SCHED_OTHER for non-privileged users..
0.045 [ Warning][Activity] Lowering scheduler type to SCHED_OTHER for non-privileged users..
   Switched to : Deployer

  This console reader allows you to browse and manipulate TaskContexts.
  You can type in an operation, expression, create or change variables.
  (type 'help' for instructions and 'ls' for context info)

    TAB completion and HISTORY is available ('bash' like)

    Use 'Ctrl-D' or type 'quit' to exit this program.

Deployer [S]>
```

If you get :
```
Real-time memory: 517904 bytes free of 524288 allocated.
0.044 [ Warning][TaskContextServer()] CTaskContext 'Deployer' could not find CORBA Naming Service.
0.044 [ Warning][TaskContextServer()] Writing IOR to 'std::cerr' and file 'Deployer.ior'
IOR:010000001f00000049444c3a5254542f636f7262612f435461736b436f6e746578743a312e300000010000000000000068000000010102000f0000003133342e3135372e31392e31353100008bd400000e000000febe3aaf5600004cbc000000000000000200000000000000080000000100000000545441010000001c00000001000000010001000100000001000105090101000100000009010100
```

Then make sure omniorb nameserver is launched (a simple reboot can also do the trick):
```sudo service omniorb4-nameserver start```

Try to launch lwr_sim inside the gazebo process:

```
roslaunch lwr_utils lwr_gazebo.launch
```

Should output ```[Info][lwr_sim] Gazebo component running```

If nothing happens, make sure you recompiled correctly the whole workspace *AFTER*
