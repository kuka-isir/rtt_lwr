Compile faster with Ccache and Distcc
=====================================

* ccache : https://ccache.samba.org/
* distcc : https://github.com/distcc/distcc

Installation
------------

.. code-block:: bash

    sudo apt install distcc* ccache

Configuration
-------------

.. code-block:: bash

    export CCACHE_PREFIX="distcc"
    export CC="ccache gcc-4.8" CXX="ccache g++-4.8"
    # If you want only distcc use this
    #export CC="distcc gcc-4.8" CXX="distcc g++-4.8"

    # List here all the known distcc servers/numer of threads
    export DISTCC_HOSTS='localhost/4 kuka-viz/6 kuka-viz2/6'


.. code-block:: bash

    # Build your workspace with
    catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver

.. tip:: Put this alias in your ``~/.bashrc``: ``alias cdistcc="catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver"``

.. warning:: The command in ``CC`` and ``CXX`` are the **commands sent** over the network. In order to avoid compiler conflicts, put the compiler version (ex: ``gcc-4.8``) !

.. note::

    If you've already built your workspace without distcc, you'll need to **clean** it first ``catkin clean -y``.
