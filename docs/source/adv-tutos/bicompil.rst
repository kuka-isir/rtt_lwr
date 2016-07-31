Bi-Compilation gnulinux/Xenomai
===============================

**Tutorial** : You are using a gnulinux computer and would like to try to build for xenomai

1.  You can install a xenomai kernel and go all xenomai. Please refer to the Xenomai section </>.
2.  You can **build** your components to xenomai and test it on another xenomai computer later.

First, get Xenomai libs :

.. code-block:: bash

    wget http://xenomai.org/downloads/xenomai/stable/xenomai-2.6.5.tar.bz2
    tar xfvj xenomai-2.6.5.tar.bz2
    cd xenomai-2.6-5
    mkdir install
    ./configure --prefix=`pwd`/install
    make -j`nproc`
    make install
    # Now Xenomai is installed in ~/xenomai-2.6-5/install

Then clean everything in your orocos workspace :

.. code-block:: bash

    cd orocos-2.9_ws/
    catkin clean -y

Then add new profiles :

.. code-block:: bash

    catkin profile add xenomai
    catkin profile add gnulinux

If you run catkin profile list :

.. code-block:: bash

    [profile] Available profiles:
    xenomai (active)
    default
    gnulinux

For Xenomai
-----------

Use the xenomai profile : ``catkin profile set xenomai``
Whatever we sourced in the bashrc, we'll force catkin to use OUR env vars.
First let's export some variables :

.. code-block:: bash

    export XENOMAI_ROOT_DIR=~/xenomai-2.6.5/install

Let's configure the workspace, the important option is --env-cache as it will save the current environment in a static file (inside orocos-2.9_ws/.catkin_tools).
It is only saving when running catkin build !

.. code-block:: bash

    catkin config --extend /opt/ros/indigo \
        --install \
        --env-cache \
        -b build-xenomai   \
        -d devel-xenomai   \
        -i install-xenomai \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_CORBA=ON -DENABLE_MQ=ON -DCORBA_IMPLEMENTATION=OMNIORB

Note that we put the build, devel and install space in a different folder so it won't collide with gnulinux builds.

Now you can build !

.. code-block:: bash

    OROCOS_TARGET=xenomai catkin build

Once it's done you'll have :

.. code-block:: bash

    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ ll
    total 44
    drwxrwxr-x 11 hoarau hoarau 4096 Jul 27 22:48 ./
    drwxrwxr-x 15 hoarau hoarau 4096 Jul 27 16:36 ../
    drwxrwxr-x 10 hoarau hoarau 4096 Jul 27 22:50 build-xenomai/
    drwxrwxr-x  3 hoarau hoarau 4096 May  2 10:13 .catkin_tools/
    drwxrwxr-x  5 hoarau hoarau 4096 Jul 27 16:57 devel-xenomai/
    drwxrwxr-x  7 hoarau hoarau 4096 Jul 27 22:50 install-xenomai/
    drwxrwxr-x 11 hoarau hoarau 4096 Jul 27 22:40 logs/
    drwxrwxr-x  4 hoarau hoarau 4096 Jun  7 11:55 src/

And inside install-xenomai/lib you'll have -xenomai libraries :

.. code-block:: bash

    lrwxrwxrwx 1 hoarau hoarau 47 Jul 27 16:58 liborocos-ocl-deployment-corba-xenomai.so -> liborocos-ocl-deployment-corba-xenomai.so.2.9.0

For gnulinux
------------

Let's use the default profile : ``catkin profile set default``
Let's configure it the standard way :

.. code-block:: bash

    catkin config --extend /opt/ros/indigo \
        --env-cache \
        -b build \
        -d devel \
        --install \
        -i install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_CORBA=ON \
        -DENABLE_MQ=ON -DCORBA_IMPLEMENTATION=OMNIORB

And build it :

.. code-block:: bash

    OROCOS_TARGET=gnulinux catkin build

We now have two version of the same libs (gnulinux/xenomai) :

.. code-block:: bash

    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ ll
    total 44
    drwxrwxr-x 11 hoarau hoarau 4096 Jul 27 22:48 ./
    drwxrwxr-x 15 hoarau hoarau 4096 Jul 27 16:36 ../
    drwxrwxr-x 11 hoarau hoarau 4096 Jul 27 22:40 build/
    drwxrwxr-x 10 hoarau hoarau 4096 Jul 27 22:50 build-xenomai/
    drwxrwxr-x  3 hoarau hoarau 4096 May  2 10:13 .catkin_tools/
    drwxrwxr-x  5 hoarau hoarau 4096 Jul 27 22:40 devel/
    drwxrwxr-x  5 hoarau hoarau 4096 Jul 27 16:57 devel-xenomai/
    drwxrwxr-x  7 hoarau hoarau 4096 Jul 27 22:41 install/
    drwxrwxr-x  7 hoarau hoarau 4096 Jul 27 22:50 install-xenomai/
    drwxrwxr-x 11 hoarau hoarau 4096 Jul 27 22:40 logs/
    drwxrwxr-x  4 hoarau hoarau 4096 Jun  7 11:55 src/

Now let's prove it works :

.. code-block:: bash

    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ catkin profile list
    [profile] Available profiles:
    xenomai (active)
    default
    gnulinux

    # Let's check the global linux variable
    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ echo $OROCOS_TARGET
    gnulinux

    # The next command will show the env variable written on the package rtt cache
    # (enabled via catkin config --env-cache)
    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ catkin build --get-env rtt | grep ORO
    typeset -x OROCOS_TARGET=xenomai
    # We have xenomai written in the cache of the rtt package !

We are on xenomai profile, the global linux OROCOS_TARGET is set to gnulinux, but the env written on each package (here rtt) is xenomai !

You can also try to launch to launch the deployer on non-xenomai kernel :

.. code-block:: bash

    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ source install-xenomai/setup.sh
    hoarau@waro-G55VW:~/ros_ws/orocos-2.9_ws$ deployer
    Xenomai: /dev/rtheap is missing
    (chardev, major=10 minor=254)
    # It doest not work, because you don't have a xenomai kernel

Final note : on kuka-rt2 (xenomai kernel) it can support both versions !
