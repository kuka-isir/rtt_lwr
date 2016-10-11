RTT/ROS/KDL Tools
=================

Available at https://github.com/kuka-isir/rtt_ros_kdl_tools

This set of tools helps you build and use a robot kinematic model via a provided URDF.

Robot Kinematic Model
---------------------

The ``ChainUtils`` class provides tools to compute forward kinematics, external torques etc.

* `ChainUtilsBase <https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils_base.hpp>`_
* `ChainUtilsBase + IK <https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils.hpp>`_


Frames of reference / ROS params
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To build the kinematic chain, we'll use the folowing ros params :

* ``/robot_description`` (the URDF uploaded)
* ``/root_link`` (default/recommended : /link_0)
* ``/tip_link`` (example : /ati_link)

All those parameters must live in the relative namespace (http://wiki.ros.org/Names) from your node to build the chain with KDL.

.. note::

    If you use the ``rtt_lwr`` tools, like the generated ``run.launch`` linked to ``lwr_utils``, the params are **automatically** sent when you'll run your controller.

.. image:: https://docs.google.com/drawings/d/1IsCpcNgwbicAzSDBb1ejsGLNkccFDNkBfcELeXQbdK8/pub?w=668&amp;h=602


Build your ChainUtils model
~~~~~~~~~~~~~~~~~~~~~~~~~~~

In your ``package.xml`` :

.. code-block:: xml

    <build_depend>rtt_ros_kdl_tools</build_depend>
    <run_depend>rtt_ros_kdl_tools</run_depend>

In your ``CMakeLists.txt`` :

.. code-block:: cmake

    # Add the rtt_ros dependency
    find_package(catkin REQUIRED COMPONENTS rtt_ros)
    # It's gonna import all orocos libraries in the package.xml

In your ``.hpp`` :

.. code-block:: cpp

    #include <rtt_ros_kdl_tools/chain_utils.hpp>

In your ``.cpp`` :

.. code-block:: cpp

    // Construct your model

    rtt_ros_kdl_tools::ChainUtils arm;

    // Initialise chain from robot_description (via ROS Params)
    // It reads /robot_description
    //          /tip_link
    //          /root_link

    arm.init();

Inverse kinematics with trac_ik
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

    // Ex : read from orocos port in an Eigen::VectorXd
    port_joint_position_in.read(jnt_pos_in)

    // Transform it to a KDL JntArray
    KDL::JntArray jnt_pos_current_kdl(jnt_pos_in.size());
    jnt_pos_current_kdl.data = jnt_pos_in;


    // Allocate the output of the Inverse Kinematics
    KDL::JntArray jnt_pos_out_kdl(jnt_pos_in.size());

    // Create an desired frame
    KDL::Frame desired_end_effector_pose(
            KDL::Rotation::RPY(-1.57,0,1.57),
            KDL::Vector(-0.2,-0.3,0.8));

    // Define some tolerances
    KDL::Twist tolerances(KDL::Vector(0.01,0.01,0.01),KDL::Vector(0.01,0.01,0.01))

    // Call the inverse function:
    // ChainUtils::cartesianToJoint(KDL::JntArray joint_seed,
    //                              KDL::Frame desired_end_effector_pose,
    //                              KDL::JntArray& return_joints,
    //                              KDL::Twist tolerances)

    if(arm.cartesianToJoint(jnt_pos_kdl,
                            desired_end_effector_pose,
                            jnt_pos_out_kdl,
                            tolerances))
    {
        log(Debug) << "Success !" << endlog();
    }
