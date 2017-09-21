Getting started with rtt_lwr 2.0
================================

The main launch file
--------------------

In lwr_utils, you'll find the main roslaunch to deploy the components :

.. code-block:: ruby

    roslaunch lwr_utils run.launch sim:=true launch_gazebo:=true


.. image:: /_static/main_launch.png
    :alt: Simple run.launch without arguments

This uploads the ``robot description`` (tools accessible via ``load_*:=true`` arguments), the ``robot_state_publisher`` ( the ``joint_state_publisher`` is done by an orocos component called ``rtt_state_publisher``), the service to spawn the robot on gazebo and a few parameters to get the robot name, namespace, tf_prefix etc.

By default, this passes the **utils.ops script** that contains a bunch of useful functions to load the robot and a few components that comes with rtt_lwr. This script is located at ``$(rospack find lwr_utils)/scripts/utils.ops``. You can change the script loaded by ``run.launch`` via the **ops_script:=** argument.

Later on, you're gonna pass your own customized script as argument :
``touch my_script.ops && nano my_script.ops`` and ``run.launch ops_script:=/path/to/my_script.ops``

.. code-block:: ruby

    import("rtt_rospack")
    runScript(ros.find("lwr_utils") + "/scripts/utils.ops")

    # loadRobot(getRobotName(),isSim(),true)
    # ....
    # Your own functions !


Writing your own deployment script (ops file)
---------------------------------------------

A typical sequence for deploying components would be :

.. code-block:: ruby

    # Load rospack to find packages in the ros workspace
    import("rtt_rospack")
    # Load the utility script into the deployer
    runScript(ros.find("lwr_utils")+"/scripts/utils.ops")
    # Load the robot
    loadRobot(getRobotName(),isSim(),true)
    # Load the state publisher for rviz visualization
    loadStatePublisher(true)

    # Then you can load your component, connect it etc.

.. note::

        Instead of creating everything **by hand**, please follow the `Controller Tutorial <controller-tuto>`_ and generate a sample project.

Available global functions :

.. code-block:: ruby

    curl --silent https://raw.githubusercontent.com/kuka-isir/rtt_lwr/rtt_lwr-2.0/lwr_utils/scripts/utils.ops | grep  global

.. code-block:: ruby

    global string getRobotName()
    global string getRobotNs()
    global string getTfPrefix()
    global bool isSim()
    global bool setRobotInitialJointConfiguration(double q0,double q1,double q2,double q3,double q4,double q5,double q6)
    global string loadKRLTool(bool start_component)
    global void setJointTorqueControlMode()
    global void setCartesianImpedanceControlMode()
    global void setJointImpedanceControlMode()
    global bool importRequiredPackages()
    global bool connectPortsFromList(string controller_name,string robot_name,strings ports_list ,ConnPolicy cp)
    global bool connectStandardPorts(string controller_name,string robot_name, ConnPolicy cp)
    global bool connectLWRPorts(string controller_name,string robot_name, ConnPolicy cp)
    global bool connectAllPorts(string controller_name,string robot_name, ConnPolicy cp)
    global string loadStatePublisher(bool start_component)
    global string loadConman()
    global bool addComponentToStateEstimation(string component_name)
    global bool addRobotToConman(string component_name)
    global bool addControllerToConman(string component_name)
    global string loadFBSched()
    global bool addControllerToFBSched(string component_name)
    global void generateGraph()
    global string loadJointTrajectoryGeneratorKDL(bool start_component)
    global string loadROSControl(bool start_component)
    global string getAtiFTSensorDataPort()
    global bool connectToAtiFTSensorPort(string comp_name,string port_name,ConnPolicy cp)
    global string loadAtiFTSensor(bool start_component)
    global string loadRobot(string robot_name,bool is_sim,bool start_component)
