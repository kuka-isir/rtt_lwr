**Update the packages**
#########################

We are periodically doing updates on the code (new tools, bug fixes etc), so keeping it up-to-date can be very useful.

To update all the packages we're going to use the ``vcs-tools`` utility (https://github.com/dirk-thomas/vcstool).

It should be already installed during the :doc:`installation </install/install-16.04-kinetic>` procedure, otherwise ``sudo apt install python-vcstool``.

Update OROCOS
-------------

.. code-block:: bash

        cd ~/isir/orocos-2.9_ws/src
        vcs pull
        # update submodules for bleeding-edge updates (OPTIONAL)
        vcs-git submodule foreach git checkout toolchain-2.9
        vcs-git submodule foreach git pull

Update RTT ROS Integration
--------------------------

.. code-block:: bash

        cd ~/isir/rtt_ros-2.9/src
        vcs pull

Update rtt_lwr
--------------

.. code-block:: bash

        cd ~/isir/lwr_ws/src
        vcs pull
        vcs-git submodule update
