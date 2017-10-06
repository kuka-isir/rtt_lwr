RTnet setup on Xenomai
======================

Official website : http://www.rtnet.org/index.html

RTnet allows you to send and receive data with very strict constraints, in a real-time environment (RTAI, Xenomai). It only works with a very limited set of ethernet cards (RTnet includes "real-time" re-written drivers) : Intel PRO/1000, 82574L, any card with r8169 (xenomai < 2.6.4 only) and others.

First make sure your followed the `Xenomai installation instructions <rtpc/xenomai.html>`_ and you are running the Xenomai kernel (uname -a).

Recommended hardware
--------------------

* `Intel Pro/1000 GT <http://amzn.eu/5AxzYzA>`_ : e1000e driver *<-- Recommended*
* `D-Link DGE-528T <http://amzn.eu/cR3yfe4>`_ : r8169s driver

Check which kernel driver you use
---------------------------------

.. code-block:: bash

    lspci -vvv -nn | grep -C 10 Ethernet

And check if the **rt_** version exists in `RTnet's drivers <https://github.com/kuka-isir/RTnet/tree/master/drivers>`_.

.. note::

    We're using a custom website that fixes compilation problems for kernel > 3.10 `source <http://sourceforge.net/p/rtnet/mailman/message/33151881//>`_.

Download
--------

.. code-block:: bash

    sudo apt install git

.. code-block:: bash

    git clone https://github.com/kuka-isir/RTnet.git

Installation
------------

We'll need the following options:

.. code-block:: bash

    * Variant
      --> Xenomai 2.1
    * Protocol Stack
      --> TCP Support (for ATI Force Sensor)
    * Drivers
      --> The driver you use (New Intel PRO/1000 in our case)
      --> Loopback (optional)
    * Add-Ons
      --> Real-Time Capturing Support (optional, for Wireshark debugging)
    * Examples
      --> RTnet Application Examples
          --> Enable (optional)


.. code-block:: bash

    sudo apt install libncurses5-dev

.. code-block:: bash

    cd RTnet
    make menuconfig
    # Configure the options below
    # Then hit ESC 2 times, save the config, and build
    make
    # Install in /usr/local/rtnet/ (default location)
    sudo make install

Configuration
-------------

The configuration file is located by default at ``/usr/local/rtnet/etc/rtnet.conf``
Take a look at `this configuration file <https://github.com/kuka-isir/rtt_lwr/blob/master/lwr_scripts/config/rtnet.conf>`_.

* **RT_DRIVER="rt_e1000e"** The driver we use (we have the Intel PRO/1000 GT)
* **REBIND_RT_NICS="0000:05:00.0 0000:06:00.0"** NIC addresses of the 2 cards we use for RTnet (you can check the NIC address typing 'lshw -C network' and looking at "bus info: pci@...". It is useful to have a fix master/slave config order (card1->robot, card2->Sensor for example).
* **IPADDR="192.168.100.101"** IP of the master (your computer). ALl the slaves will send/receive to/from master IP.
* **NETMASK="255.255.255.0"** The other slave will have IPs 192.168.100.XXX.
* **RT_LOOPBACK="no"** Not used now. Might be useful to use it somehow.
* **RT_PROTOCOLS="udp packet tcp"** Robot sends via UDP, ATI Sensor via TCP for config, UDP otherwise.
* **RTCAP="yes"** To debug with Wireshark
* **TDMA_CYCLE="450"** and **TDMA_OFFSET="50"** Data from robot/ sensor takes about 350us to receive (using Wireshark).


Allow non-root users
--------------------

To allow commands like ``rtnet start`` etc to be used without ``sudo``, we will use ``visudo``.
We remove password in certain commands *only for people in the xenomai group*.

.. code-block:: bash

    sudo visudo
    # then add the following at the end
    %xenomai ALL=(root) NOPASSWD:/sbin/insmod
    %xenomai ALL=(root) NOPASSWD:/sbin/rmmod
    %xenomai ALL=(root) NOPASSWD:/sbin/modprobe
    %xenomai ALL=(root) NOPASSWD:/bin/echo
    %xenomai ALL=(root) NOPASSWD:/bin/mknod
    %xenomai ALL=(root) NOPASSWD:/usr/bin/service
    %xenomai ALL=(root) NOPASSWD:/usr/sbin/service
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/rtcfg
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/rtifconfig
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/rtiwconfig
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/rtnet
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/rtping
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/rtroute
    %xenomai ALL=(root) NOPASSWD:/usr/local/rtnet/sbin/tdmacfg


Test your installation
----------------------

Using the test script
~~~~~~~~~~~~~~~~~~~~~

A launch script can be found `here <https://github.com/kuka-isir/rtt_lwr/blob/master/lwr_scripts/scripts/rtnet/>`_.
Adjust the following settings to your needs :

* SLAVES="192.168.100.102 192.168.100.103"
* SLAVES_NAMES="Kuka ATISensor"

Then to use it :

.. code-block:: bash

    ./path/to/script/rtnet start

Manually
~~~~~~~~~

.. code-block:: bash

    cd /usr/local/rtnet/sbin
    # Start the rt kernel drivers
    sudo ./rtnet start
    # Bringup connection
    sudo ./rtifconfig rteth0 up 192.168.100.101 netmask 255.255.255.0
    # Bringup slaves
    sudo ./rtroute solicit 192.168.100.102 dev rteth0
    # Ping Slave
    sudo ./rtping 192.168.100.102
    # Stop everything
    sudo ./rtnet stop


.. note::

    You might have to remove the non-rt kernel driver before rtnet start :

    .. code-block:: bash

        sudo rmmod e1000e
        sudo ./rtnet start

.. note::

    You should see rt_e1000e as the kernel driver currently used

    .. code-block:: bash

        lspci -vvv -nn | grep -C 10 Ethernet
        lsmod | grep rt_


Use RTnet in C++
----------------

The API is the same as regular socket in C, except that the functions start with ``rt_*``.
To make sure it compiles on every platform, add the following to your headers :

.. code-block:: cpp

    #ifndef HAVE_RTNET

    // Rename the standard functions
    // And use theses ones to be RTnet-compatible when available

    #define rt_dev_socket     socket
    #define rt_dev_setsockopt setsockopt
    #define rt_dev_bind       bind
    #define rt_dev_recvfrom   recvfrom
    #define rt_dev_sendto     sendto
    #define rt_dev_close      close
    #define rt_dev_connect    connect

    #else
    // Use RTnet in Xenomai
    #include <rtdm/rtdm.h>
    #endif

And in your CMakeLists.txt (example) :

.. code-block:: cmake

    # Add the path to the FindRTnet.cmake folder
    # Let's assume you put it in /path/to/project/cmake
    list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

    if($ENV{OROCOS_TARGET} STREQUAL "xenomai")
      find_package(RTnet)
      if(NOT ${RTnet_FOUND})
        message(ERROR "RTnet cannot be used without Xenomai")
      else()
        message(STATUS "RTnet support enabled")
        set_property(TARGET ${TARGET_NAME} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
      endif()
    endif()


.. note:: `FindRTnet.cmake can be found here <https://github.com/kuka-isir/ati_sensor/tree/master/cmake/Modules>`_.
