Xenomai 2.6.5 on Ubuntu 14.04
#############################

.. note::

    Nvidia Drivers are NOT supported (creates a lot of interruptions that breaks the real-time).
    Please consider removing the dedicated graphic card and use the integrated graphics (Intel HD graphics).

Download Xenomai 2.6.5
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    wget http://xenomai.org/downloads/xenomai/stable/xenomai-2.6.5.tar.bz2
    tar xfvj xenomai-2.6.5.tar.bz2


Download Linux kernel 3.18.20
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    wget https://www.kernel.org/pub/linux/kernel/v3.x/linux-3.18.20.tar.gz
    tar xfv linux-3.18.20.tar.gz


Configuration
~~~~~~~~~~~~~

Prepare the kernel
------------------

.. code-block:: bash

    sudo apt install kernel-package

Patch the Linux kernel with Xenomai ipipe patch.
-----------------------------------------------

.. code-block:: bash

    cd linux-3.18.20
    ../xenomai-2.6.5/scripts/prepare-kernel.sh

Press Enter to use the default options.

Configure the kernel
--------------------

Now it's time to configure :

.. code-block:: bash

    make xconfig

Some guidelines to configure the linux kernel:

.. code-block:: text

    Required Options:
    * Real-time sub-system
      --> Xenomai (Enable)
      --> Nucleus (Enable)
    * Power management and ACPI options
      --> Run-time PM core functionality (Disable)
      --> ACPI (Advanced Configuration and Power Interface) Support
          --> Processor (Disable)
      --> CPU Frequency scaling
          --> CPU Frequency scaling (Disable)
      --> CPU idle
          --> CPU idle PM support (Disable)
    * Pocessor type and features
      --> Processor family
          --> Core 2/newer Xeon (if \"cat /proc/cpuinfo | grep family\" returns 6, set as Generic otherwise)

    Optional Options (RECOMMENDED):
    * General setup
      --> Local version - append to kernel release: -xenomai-2.6.5
      --> Timers subsystem
          --> High Resolution Timer Support (Verify you have HRT ON)
    * Pocessor type and features
      --> Processor family
          --> SMT (Hyperthreading) scheduler support (Disable)
          --> Preemption Model
              --> Voluntary Kernel Preemption (Desktop)
    * Power management and ACPI options
      --> Memory power savings
          --> Intel chipset idle memory power saving driver

.. warning::

    For OROCOS, we need to increase the amount of ressources available for Xenomai tasks, otherwise we might hit the limits quickly as we add multiples components/ports etc. http://www.orocos.org/forum/orocos/orocos-users/orocos-limits-under-xenomai

    .. code-block:: bash

        * Real-time sub-system
          --> Number of registry slots
              --> 4096
          --> Size of the system heap
              --> 2048 Kb
          --> Size of the private stack pool
              --> 1024 Kb
          --> Size of private semaphores heap
              --> 48 Kb
          --> Size of global semaphores heap
              --> 48 Kb

Save the config and close the gui.

Compile the kernel (make debians)
---------------------------------

Now it's time to compile.

.. code-block:: bash

    CONCURRENCY_LEVEL=$(nproc) make-kpkg --rootcmd fakeroot --initrd kernel_image kernel_headers

Take a coffee and come back in 20min.

Compile faster with distcc
--------------------------

.. code-block:: bash

    MAKEFLAGS="CC=distcc" BUILD_TIME="/usr/bin/time" CONCURRENCY_LEVEL=$(distcc -j) make-kpkg --rootcmd fakeroot --initrd kernel_image kernel_headers

Install the kernel
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    cd ..
    sudo dpkg -i linux-headers-3.18.20-xenomai-2.6.5_3.18.20-xenomai-2.6.5-10.00.Custom_amd64.deb linux-image-3.18.20-xenomai-2.6.5_3.18.20-xenomai-2.6.5-10.00.Custom_amd64.deb


Configure GRUB
~~~~~~~~~~~~~~

Minimal config (``xeno_nucleus.xenomai_gid=128 xenomai.allowed_group=128`` allows non-root xenomai tasks):

.. code-block:: bash

    GRUB_DEFAULT=saved
    GRUB_SAVEDEFAULT=true
    #GRUB_HIDDEN_TIMEOUT=0
    GRUB_HIDDEN_TIMEOUT_QUIET=true
    GRUB_TIMEOUT=5
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash xeno_nucleus.xenomai_gid=128 xenomai.allowed_group=128"
    GRUB_CMDLINE_LINUX=""


.. tip:: `noapic`` option might be added if the screen goes black at startup

If you have an Intel HD Graphics integrated GPU :

.. code-block:: bash

    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash i915.i915_enable_rc6=0 i915.powersave=0 noapic xeno_nucleus.xenomai_gid=128 xenomai.allowed_group=128"
    # This removes powersavings from the graphics, that creates disturbing interruptions.


Update GRUB and reboot

.. code-block:: bash

    sudo update-grub
    sudo reboot

Allow non-root users
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo addgroup xenomai
    sudo addgroup root xenomai
    sudo usermod -a -G xenomai $USER


Install Xenomai libraries
-------------------------

.. code-block:: bash

    cd xenomai-2.6.5/
    ./configure
    make -j$(nproc)
    sudo make install


Update your bashrc

.. code-block:: bash

    echo '
    #### Xenomai
    export XENOMAI_ROOT_DIR=/usr/xenomai
    export XENOMAI_PATH=/usr/xenomai
    export PATH=$PATH:$XENOMAI_PATH/bin
    export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$XENOMAI_PATH/lib/pkgconfig
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$XENOMAI_PATH/lib
    export OROCOS_TARGET=xenomai
    ' >> ~/.bashrc

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

    To get pertinent results, you need to **stress** your system.
    to do so, you can use ``stress`` or ``dohell`` from the ``apt``.

    .. code-block:: bash

        # Using stress
        stress -v -c 8 -i 10 -d 8

Negative latency issues
-----------------------

You need to be in root ``sudo -s``, then you can set values to the latency calibration variable :

.. code-block:: bash

    $ echo 0 > /proc/xenomai/latency
    (run the native calibration test)
    
    If the minimum latency value is positive, then:
    $ echo abs(min-latency-value) > /proc/xenomai/latency

Source : https://xenomai.org/pipermail/xenomai/2007-May/009063.html
