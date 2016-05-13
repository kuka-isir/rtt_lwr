#####################
**How to update rtt_lwr**
#####################


We are periodically doing updates on the code (gains update, bug fixes etc), so keeping it up-to-date can be very useful.

To update all the packages we're going to use the ``vcs-tools`` utility (https://github.com/dirk-thomas/vcstool).

It should be already installed during the :doc:`installation </install/install>` procedure, otherwise ``sudo apt-get install python-vcs-tool``.

.. code-block:: bash

        cd ~/lwr_ws/src
        # Launches a git pull on every git repos it finds in current directory
        vcs pull
        # update submodules as well (lwr_hardware and rtt_lwr_sim)
        vcs-git submodule update
