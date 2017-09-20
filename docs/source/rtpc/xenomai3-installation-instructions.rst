Xenomai 3.0.5 on Ubuntu 16.04
=============================

Official documentation is available at https://xenomai.org/installing-xenomai-3-x/

Get the linux kernel
--------------------

.. code-block:: bash

    wget https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.9.38.tar.gz
    tar xf linux-4.9.38.tar.gz

.. note::

    We chose 4.9.38 because it is the latest kernel compatible with xenomai.
    You can find the patches at https://xenomai.org/downloads/ipipe/


Get Xenomai 3.0.5
-----------------

.. code-block:: bash

    wget https://xenomai.org/downloads/xenomai/stable/xenomai-3.0.5.tar.bz2
    tar xf xenomai-3.0.5.tar.bz2

Apply the Xenomai patch
-----------------------

We assume you are building an **x86_64** kernel (32/64bits).

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

.. warning:: Unlike xenomai 2.x, RTnet has to be built in the kernel !


Build the Real-Time kernel
--------------------------

.. code-block:: bash

    sudo apt install kernel-package
    CONCURRENCY_LEVEL=$(nproc) make-kpkg --rootcmd fakeroot --initrd kernel_image kernel_headers


.. code-block:: bash

    cd ..
    sudo dpkg -i linux-headers-4.9.38-xenomai-3.0.5_4.9.38-xenomai-3.0.5-10.00.Custom_amd64.deb linux-image-4.9.38-xenomai-3.0.5_4.9.38-xenomai-3.0.5-10.00.Custom_amd64.deb


Installing Xenomai 3.0.5
------------------------

On Mercury Core
~~~~~~~~~~~~~~~

.. code-block:: bash

    ./configure --with-pic --with-core=mercury --enable-smp --disable-tls

On Cobalt Core
~~~~~~~~~~~~~~

.. code-block:: bash

    ./configure --with-pic --with-core=cobalt --enable-smp --disable-tls --enable-dlopen-libs

GRUB_DEFAULT=saved
GRUB_SAVEDEFAULT=true
