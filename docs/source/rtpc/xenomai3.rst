(Beta) Xenomai 3.0.5 on Ubuntu 14.04/16.04
==========================================

These instructions demonstrates how to build a Cobalt Core for Xenomai 3.0.5.

Official documentation is available at https://xenomai.org/installing-xenomai-3-x/

Recommended Hardware
~~~~~~~~~~~~~~~~~~~~

* **Intel/AMD Processor i5/i7** (< 2016 is recommended to guarantee full 16.04 support)
* **Dedicated Ethernet controller for RTnet**, with either e1000e/e1000/r8169 driver (Intel PRO/1000 GT recommended)


.. warning::

    Nvidia/Ati Drivers are NOT supported (creates a lot of interruptions that breaks the real-time constraints).
    Please consider removing the dedicated graphic card and use the integrated graphics (Intel HD graphics).


Get the linux kernel
--------------------

.. code-block:: bash

    wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.9.38.tar.gz
    tar xf linux-4.9.38.tar.gz

.. note::

    We chose 4.9.38 because it is the latest kernel compatible with xenomai 3.0.5.
    You can find the patches at https://xenomai.org/downloads/ipipe/


Get Xenomai 3.0.5
-----------------

.. code-block:: bash

    wget https://xenomai.org/downloads/xenomai/stable/xenomai-3.0.5.tar.bz2
    tar xf xenomai-3.0.5.tar.bz2

Apply the Xenomai patch
-----------------------

We assume you are building an **x86_64** kernel (64bits).

.. code-block:: bash

  cd linux-4.9.38
  wget https://xenomai.org/downloads/ipipe/v4.x/x86/ipipe-core-4.9.38-x86-3.patch
  ../xenomai-3.0.5/scripts/prepare-kernel.sh --arch=x86_64 --ipipe=ipipe-core-4.9.38-x86-3.patch


Configure the kernel
--------------------

Take the actual working config :

.. code-block:: bash

    yes "" | make oldconfig

Gui version :

.. code-block:: bash

    make xconfig

Or GTK+ version :

.. code-block:: bash

    sudo apt install gtk+-2.0 glib-2.0 libglade2-dev
    make gconfig

Or without gui :

.. code-block:: bash

    sudo apt install libncurses5-dev
    make menuconfig


Some guidelines to configure the linux kernel:

.. code-block:: text

    Recommended options:

    * General setup
      --> Local version - append to kernel release: -xenomai-3.0.5
      --> Timers subsystem
          --> High Resolution Timer Support (Enable)
    * Xenomai/cobalt
      --> Sizes and static limits
        --> Number of registry slots (512 --> 4096)
        --> Size of system heap (Kb) (512 --> 4096)
        --> Size of private heap (Kb) (64 --> 256)
        --> Size of shared heap (Kb) (64 --> 256)
        --> Maximum number of POSIX timers per process (128 --> 512)
      --> Drivers
        --> RTnet
            --> RTnet, TCP/IP socket interface (Enable)
                --> Drivers New intel(R) PRO/1000 PCIe (Enable)
            --> Add-Ons
                --> Real-Time Capturing Support (Enable)
    * Power management and ACPI options
      --> CPU Frequency scaling
          --> CPU Frequency scaling (Disable)
      --> ACPI (Advanced Configuration and Power Interface) Support
          --> Processor (Disable)
      --> CPU Idle
          --> CPU idle PM support (Disable)
    * Pocessor type and features
      --> Enable maximum number of SMP processors and NUMA nodes (Disable)
      // Ref : http://xenomai.org/pipermail/xenomai/2017-September/037718.html
      --> Processor family
          --> Core 2/newer Xeon (if "cat /proc/cpuinfo | grep family" returns 6, set as Generic otherwise)
      // Xenomai will issue a warning about CONFIG_MIGRATION, disable those in this order
      --> Transparent Hugepage Support (Disable)
      --> Allow for memory compaction (Disable)
      --> Contiguous Memory Allocation (Disable)
      --> Allow for memory compaction
        --> Page Migration (Disable)
    * Device Drivers
      --> Staging drivers
          --> Unisys SPAR driver support
             --> Unisys visorbus driver (Disable)

.. warning:: Unlike xenomai 2.x, RTnet has to be built **in** the kernel. Make sure to choose the drivers correctly.


Build the Real-Time kernel
--------------------------

.. code-block:: bash

    sudo apt install kernel-package
    CONCURRENCY_LEVEL=$(nproc) make-kpkg --rootcmd fakeroot --initrd kernel_image kernel_headers

Compile faster with distcc
--------------------------

If you have distcc servers setup and a fast network, you can speed up drastically the building speed.

.. code-block:: bash

    MAKEFLAGS="CC=/usr/lib/distcc/gcc-5" make-kpkg -j$(distcc -j) --rootcmd fakeroot --initrd kernel_image kernel_headers

.. note::

    Always set gcc version for distcc so that the server can figure out which one to choose (it may have gcc-4.8 by default)

.. code-block:: bash

    cd ..
    sudo dpkg -i linux-headers-4.9.38-xenomai-3.0.5_4.9.38-xenomai-3.0.5-10.00.Custom_amd64.deb linux-image-4.9.38-xenomai-3.0.5_4.9.38-xenomai-3.0.5-10.00.Custom_amd64.deb


Allow non-root users
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo addgroup xenomai --gid 1234
    sudo addgroup root xenomai
    sudo usermod -a -G xenomai $USER

.. tip:: If the addgroup command fails (ex: GID ``xenomai`` is already in use), change it to a different random value, and report it in the next section.

Configure GRUB and reboot
~~~~~~~~~~~~~~~~~~~~~~~~~

Edit the grub config :

.. code-block:: bash

    sudo nano /etc/default/grub

.. code-block:: bash

    GRUB_DEFAULT="Advanced options for Ubuntu>Ubuntu, with Linux 4.9.38-xenomai-3.0.5"
    #GRUB_DEFAULT=saved
    #GRUB_SAVEDEFAULT=true
    # Comment the following lines
    #GRUB_HIDDEN_TIMEOUT=0
    #GRUB_HIDDEN_TIMEOUT_QUIET=true
    GRUB_TIMEOUT=5
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash xenomai.allowed_group=123"
    GRUB_CMDLINE_LINUX=""

.. note::

    Please note the xenomai group (here 1234) should match what you set above (allow non-root users).

.. tip:: ``noapic`` option might be added if the screen goes black at startup and you can't boot.

If you have an Intel HD Graphics integrated GPU (any type) :

.. code-block:: bash

    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash i915.enable_rc6=0 i915.powersave=0 noapic xenomai.allowed_group=1234"
    # This removes powersavings from the graphics, that creates disturbing interruptions.

If you have an Intel **Skylake** (2015 processors), you ``need`` to add nosmap to fix the latency hang (https://xenomai.org/pipermail/xenomai/2016-October/036787.html) :

.. code-block:: bash

    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash i915.enable_rc6=0 i915.powersave=0 xeno_nucleus.xenomai_gid=1234 nosmap"

Update GRUB and reboot

.. code-block:: bash

    sudo update-grub
    sudo reboot


Installing Xenomai 3.0.5 User space libraries
---------------------------------------------

First, make sure that you are running the cobalt kernel :

.. code-block:: bash

    uname -a
    # Should return Linux waro-rt 4.9.38-xenomai-3.0.5 #2 SMP Wed Sep 20 16:00:12 CEST 2017 x86_64 x86_64 x86_64 GNU/Linux
    dmesg | grep Xenomai
    # [    1.417024] [Xenomai] scheduling class idle registered.
    # [    1.417025] [Xenomai] scheduling class rt registered.
    # [    1.417045] [Xenomai] disabling automatic C1E state promotion on Intel processor
    # [    1.417055] [Xenomai] SMI-enabled chipset found, but SMI workaround disabled
    # [    1.417088] I-pipe: head domain Xenomai registered.
    # [    1.417704] [Xenomai] allowing access to group 1234
    # [    1.417726] [Xenomai] Cobalt v3.0.5 (Sisyphus's Boulder) [DEBUG]

.. code-block:: bash

    cd xenomai-3.0.5
    ./configure --with-pic --with-core=cobalt --enable-smp --disable-tls --enable-dlopen-libs --disable-clock-monotonic-raw
    make -j`nproc`
    sudo make install
    # --disable-clock-monotonic-raw : http://xenomai.org/pipermail/xenomai/2017-September/037695.html


Prevent Gazebo compiling issues (Hack)
-------------------------------

Gazebo won't compile because of some conflicting macros ( clz() ) present in libtbb and libcobalt.
Remove this macro from xenomai to hack-fix it. It is only used in xenomai internals so won't cause any issue in user-land. 

.. code-block:: bash
    
    # http://xenomai.org/pipermail/xenomai/2017-September/037729.html
    sudo sed -i 's/clz/__clz/g' /usr/xenomai/include/boilerplate/compiler.h


Update your bashrc
------------------

.. code-block:: bash

    echo '
    ### Xenomai
    export XENOMAI_ROOT_DIR=/usr/xenomai
    export XENOMAI_PATH=/usr/xenomai
    export PATH=$PATH:$XENOMAI_PATH/bin
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$XENOMAI_PATH/lib/pkgconfig
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$XENOMAI_PATH/lib
    export OROCOS_TARGET=xenomai
    ' >> ~/.xenomai_rc

    echo 'source ~/.xenomai_rc' >> ~/.bashrc
    source ~/.bashrc

Test your installation
----------------------

.. code-block:: bash

    xeno latency

This loop will allow you to monitor a xenomai latency. Here's the output for a i7 4Ghz :

.. code-block:: bash

    == Sampling period: 100 us
    == Test mode: periodic user-mode task
    == All results in microseconds
    warming up...
    RTT|  00:00:01  (periodic user-mode task, 100 us period, priority 99)
    RTH|----lat min|----lat avg|----lat max|-overrun|---msw|---lat best|--lat worst
    RTD|      0.174|      0.464|      1.780|       0|     0|      0.174|      1.780
    RTD|      0.088|      0.464|      1.357|       0|     0|      0.088|      1.780
    RTD|      0.336|      0.464|      1.822|       0|     0|      0.088|      1.822
    RTD|      0.342|      0.464|      1.360|       0|     0|      0.088|      1.822
    RTD|      0.327|      0.462|      2.297|       0|     0|      0.088|      2.297
    RTD|      0.347|      0.463|      1.313|       0|     0|      0.088|      2.297
    RTD|      0.314|      0.464|      1.465|       0|     0|      0.088|      2.297
    RTD|      0.190|      0.464|      1.311|       0|     0|      0.088|      2.297


.. tip::

    To get pertinent results, you need to **stress** your system while running the latency test. The latency has to be stable even if the system is under load.

    .. code-block:: bash

        sudo apt install stress
        # Using stress
        stress -v -c 8 -i 10 -d 8

Negative latency issues
-----------------------

You need to be in root ``sudo -s``, then you can set values to the latency calibration variable in **nanoseconds**:

.. code-block:: bash

    $ echo 0 > /proc/xenomai/latency
    # Now run the latency test

    # If the minimum latency value is positive,
    # then get the lowest value from the latency test (ex: 0.088 us)
    # and write it to the calibration file ( here you have to write 88 ns) :
    $ echo my_super_value_in_ns > /proc/xenomai/latency

Source : https://xenomai.org/pipermail/xenomai/2007-May/009063.html
