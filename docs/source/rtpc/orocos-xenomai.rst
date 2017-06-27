#####################
OROCOS RTT on Xenomai
#####################

Orocos RTT is fully compatible with Xenomai, but it needs to be built *from source*.

Please follow the `installation instructions </install/install-16.04-kinetic.html>`_ to build OROCOS.

The only difference is the need to set ``export OROCOS_TARGET=xenomai`` before compiling.

.. code-block:: bash

    echo 'export OROCOS_TARGET=xenomai' >> ~/.bashrc
    source ~/.bashrc
    # Now you can build orocos with the standard instructions
    # Make sure you cleaned the workspace first !


.. note:: Xenomai has to be setup correctly prior to building OROCOS.

.. note:: Remember to ``clean`` your workspace prior to compiling (``catkin clean -y``)