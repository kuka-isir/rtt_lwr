Compile faster with Ccache and Distcc
=====================================

* ccache : https://ccache.samba.org/
* distcc : https://github.com/distcc/distcc

.. note:: You need to be connected to a high speed network to actually improve your build time

Installation on hosts and clients
---------------------------------

.. code-block:: bash

    sudo apt install distcc* ccache distccmon-gnome

Build server configuration
--------------------------

On every build computer, you need to install ``distcc`` and ``ccache`` and update the configuration :

.. code-block:: bash

    sudo nano /etc/default/distcc

Then set he following variables :

.. code-block:: bash

    STARTDISTCC="true"

    #
    # Which networks/hosts should be allowed to connect to the daemon?
    # You can list multiple hosts/networks separated by spaces.
    # Networks have to be in CIDR notation, f.e. 192.168.1.0/24
    # Hosts are represented by a single IP Adress
    #
    # ALLOWEDNETS="127.0.0.1"
    # This means allow from myself and computers from 192.168.1.0 to 192.168.1.255
    ALLOWEDNETS="127.0.0.1 192.168.1.0/24"

Restart the distcc deamon and you are all setup ! ``sudo service distcc restart``.

Client configuration
--------------------

.. code-block:: bash

    export CCACHE_PREFIX="distcc"
    # Use gcc 4.8 on 14.04
    export CC="ccache gcc-4.8" CXX="ccache g++-4.8"
    # Use gcc 5 on 16.04
    export CC="ccache gcc-5" CXX="ccache g++-5"
    # If you want only distcc use this
    #export CC="distcc gcc-4.8" CXX="distcc g++-4.8"
    #export CC="distcc gcc-5" CXX="distcc g++-5"

    # List here all the known distcc servers/number of threads
    export DISTCC_HOSTS='localhost/4 kuka-viz/6 kuka-viz2/6'


.. code-block:: bash

    # Build your workspace with
    catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver

.. tip:: Put this alias in your ``~/.bashrc``: ``alias cdistcc="catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver"``

.. warning:: The command in ``CC`` and ``CXX`` are the **commands sent** over the network. In order to avoid compiler conflicts, put the compiler version (ex: ``gcc-4.8``) !


Building a catkin workspace with Distcc
---------------------------------------

If you've already built your workspace without distcc, you'll need to **clean** it first `catkin clean -y`.
Then, verify you have the variables set correctly (as above) : ``echo $CXX && echo $CC``.

Now you can use ``catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver`` to build your workspace.

.. tip:: You can use ``distccmon-gnome`` to visualize the distribution.

.. image:: /_static/distcc.png
