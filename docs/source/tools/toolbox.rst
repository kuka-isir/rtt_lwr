RTT/ROS/KDL Tools
=================

Available at https://github.com/kuka-isir/rtt_ros_kdl_tools

This set of tools helps you build and use a robot kinematic model via a provided URDF.

Robot Kinematic Model
---------------------

The `ChainUtils <https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils.hpp>`_ class provides tools to compute forward kinematics, external torques etc. It devrives from `ChainUtilsBase <https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils_base.hpp>`_ and adds **non-realtime** *inverse kinematics* with `trac_ik <https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils_base.hpp>`_.


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

Internal Model
~~~~~~~~~~~~~~

.. graphviz::
    
    digraph example {
       graph [rankdir=LR];
       node[shape=box];
       q[label="Joint Position q"]
       qd[label="Joint Velocity dq"]
       subgraph cluster_0 {
           style=filled;
           href="https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils.hpp";
           target="_top";
           color=lightgrey;
           label="ChainUtils arm"
           node [style=filled,color=white];
           model[label="setState(q,dq)\nupdateModel()"]
       }
       
       q -> model
       qd -> model
       model -> getSegmentPosition
       model -> getSegmentVelocity
       model -> getSegmentJacobian
       model -> getSegmentJdot
       model -> getSegmentJdotQdot
    }

You need to tell ``ChainUtils`` the state of the robot and compute some internal model variables.

.. code-block:: c++

    // Ex : read from orocos port in an Eigen::VectorXd
    port_joint_position_in.read(jnt_pos_in); // Check if RTT::NewData
    port_joint_velocity_in.read(jnt_vel_in); // Check if RTT::NewData

    // Feed the internal state
    arm.setState(jnt_pos_in,jnt_vel_in);
    // Update the internal model
    arm.updateModel();


Forward kinematics
~~~~~~~~~~~~~~~~~~

.. graphviz::

     digraph example {
        graph [rankdir=LR];
        node[shape=box];
        q[label="Joint Positions\nq"]
        subgraph cluster_0 {
            style=filled;
            href="https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils.hpp";
            target="_top";
            color=lightgrey;
            node [style=filled,color=white];
            fk[label="Forward Kinematics\nF(q)"]
            label="ChainUtils arm"
        }
        
        x[label="Segment Position w.r.t root link\nX"]
        q -> fk -> x
     }

.. code-block:: c++

    // Ex : read from orocos port in an Eigen::VectorXd
    port_joint_position_in.read(jnt_pos_in); // Check if RTT::NewData
    port_joint_velocity_in.read(jnt_vel_in); // Check if RTT::NewData

    // Feed the internal state
    arm.setState(jnt_pos_in,jnt_vel_in);
    // Update the internal model
    arm.updateModel();

    // Ex : get a specific segment position
    KDL::Frame& X = arm.getSegmentPosition("link_7");
    
    // Ex : get the 5th segment
    KDL::Frame& X = arm.getSegmentPosition(5);
    
    // Get Root Link
    std::string root_link = arm.getRootSegmentName();


Inverse kinematics with trac_ik
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. graphviz::

     digraph example {
        graph [rankdir=LR];
        node[shape=box];
        x[label="Tip Link Position\nX"]
        subgraph cluster_0 {
            style=filled;
            href="https://github.com/kuka-isir/rtt_ros_kdl_tools/blob/master/include/rtt_ros_kdl_tools/chain_utils.hpp";
            target="_top";
            color=lightgrey;
            node [style=filled,color=white];
            ik[label="Inverse Kinematics\nF(X)"]
            label="ChainUtils arm"
        }
        
        q[label="Joint Positions\nq"]
        x -> ik -> q
     }
     
.. code-block:: cpp

    // Ex : read from orocos port in an Eigen::VectorXd
    port_joint_position_in.read(jnt_pos_in); // Check if RTT::NewData
    port_joint_velocity_in.read(jnt_vel_in); // Check if RTT::NewData

    // Feed the internal state
    arm.setState(jnt_pos_in,jnt_vel_in);
    // Update the internal model
    arm.updateModel();

    // Transform it to a KDL JntArray
    KDL::JntArray joint_seed(jnt_pos_in.size());
    joint_seed.data = jnt_pos_in;


    // Allocate the output of the Inverse Kinematics
    KDL::JntArray return_joints(jnt_pos_in.size());

    // Create an desired frame
    KDL::Frame desired_end_effector_pose(
        KDL::Rotation::RPY(-1.57,0,1.57), // Rotation rad
        KDL::Vector(-0.2,-0.3,0.8));      // Position x,y,z in meters

    // Define some tolerances
    KDL::Twist tolerances(
        KDL::Vector(0.01,0.01,0.01),   // Tolerance x,y,z in meters
        KDL::Vector(0.01,0.01,0.01));  // Tolerance Rx,Rz,Rz in rad

    // Call the inverse function
    if(arm.cartesianToJoint(joint_seed,
                            desired_end_effector_pose,
                            return_joints,
                            tolerances))
    {
        log(Debug) << "Success ! Result : "
        <<return_joints.data.transpose() << endlog();
    }
    
.. warning::
    
    ``trac-ik`` is **not realtime-safe** and therefore should not be used in ``updateHook()``
