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

Please follow [this installation script](https://github.com/kuka-isir/lwr_setup). It will install ROS Indigo, Moveit, Gazebo7, and OROCOS

