Gazebo Synchronization using FBSched
====================================

As an example you can launch ``roslaunch rtt_joint_traj_generator_kdl run.launch sim:=true``.
Then in the deployer's console :

.. code-block:: ruby

    # Then make sure gazebo follows its executionEngine
    gazebo.use_rtt_sync = true
    # load the fbs component (default 1Khz)
    loadFBSched()

    # Set a 1Hz period for the test
    fbs.setPeriod(1.0)

    # Then for each component
    addComponentToFBSched("gazebo")
    # This one is not necessary as it is gazebo's slave
    addComponentToFBSched("lwr_sim")
    # The controller
    addComponentToFBSched("lwr_sim_traj_kdl")
    # Just for the fun of it
    addComponentToFBSched("lwr_sim_state_pub")
    addComponentToFBSched("lwr_sim_krl_tool")

    # Configure and start FBSched
    fbs.configure()
    fbs.start()

.. note::

    When "fbs" starts, gazebo catches up with all the world's callbacks, so it might jump forward in time.
    If you put this code in you ops, and pause gazebo, then you won't have any issue.
