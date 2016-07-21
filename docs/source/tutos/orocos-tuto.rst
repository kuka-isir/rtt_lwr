

.. image:: /_static/orocos-logo.png
    :align: center

Orocos tutorial
===============

Orocos Basics
-------------

First, make you have the lwr workspace loaded ``source ~/lwr_ws/devel/setup.bash``

.. note::

    Put this source file in your .bashrc :
    ``echo `source ~/lwr_ws/devel/setup.bash` >> ~/.bashrc``

Official documentation about the Orocos RealTime Toolkit (RTT) can be found here :
http://www.orocos.org/stable/documentation/rtt/v2.x/doc-xml/orocos-components-manual.html

.. highlights::

    The [orocos] Toolchain allows setup, distribution and the building of real-time software components.
    It is sometimes refered to as 'middleware' because it sits between the application and the Operating System.
    It takes care of the real-time communication and execution of software components.

All components are 'deployed' using a single executable called the 'deployer'. The main deployer has the ability to load component, connect them to exchange data, start, stop, change their rate etc. To launch it just type in a terminal :

.. code::

    deployer


.. image:: /_static/deployer.png

Example of a simple 2 components deployement :

.. code-block:: ruby

    # Let's load a built-in orocos component
    loadComponent("hello","OCL::HelloWorld") # calls the constructor
    # If you want to see its ports, properties, attribues :
    ls hello
    # You can see :
    # Data Flow Ports:
    # Out(U)      string the_results    =>
    #  In(U)      string the_buffer_port <= ( use 'the_buffer_port.read(sample)'
    # to read a sample from this port)
    # It means it has 1 input port and 1 output port
    #
    # We'll build another component of the same type
    loadComponent("hello2","OCL::HelloWorld")

    # Let's connect their interface
    # It's a bi-laterial connection that allow hello1 to connect with hello2's
    # ports, attributes etc, and also hello2 to get data from hello
    connectPeers("hello","hello2")

    # Connect ports
    connect("hello.the_buffer_port","hello2.the_results",ConnPolicy())
    # The last argument ConnPolicy() is a structure that contains the way
    # to send data from one component to another. Default is "DATA"

    # Let's run everything

    # Setting the activity of
    # "hello"
    # to a period of 0.1 seconds (10Hz)
    # with the thread priority of 10 (0..99)
    # It's a standard linux thread
    setActivity("hello",0.1,10,ORO_SCHED_OTHER)

    setActivity("hello2",0.2,25,ORO_SCHED_OTHER)

    # call configureHook()
    hello.configure() #  registered as an 'operation' called 'configure'
     # call updateHook()
    hello.start()

    hello2.configure()
    hello2.start()

    # Note that parenthesis are not required for void arguments

    # Let's see the data :

    hello.the_buffer_port.last
    # It will show
    # "Hello World !"

.. tip::

    Open a ``deployer`` and copy/paste the lines one by one to test.


For further documentation, please refer to the `Orocos Builder's Manual`_.

Orocos - ROS bridge
--------------------

.. image:: /_static/rosorg-logo1.png

All the magic is done by rtt_ros_integration https://github.com/orocos/rtt_ros_integration.
Basically every ROS function that you might be used to call in regular rosnode has been wrapped for orocos to be Real-Time Safe.

Most used features :

- Transform the deployer into a ROS node (rtt_rosnode)
- Connect an Orocos port to a ROS topic (rtt_roscomm)
- Connect an Orocos operation to a ROS service (rtt_roscomm)
- Map Orocos Parameters with the ROS parameter server (rtt_rosparam)
- Get the  clock from ros (rtt_rosclock)

Custom Orocos Components with Catkin
------------------------------------

Now let's build our own Orocos Component (Very simple one with no ports, operation nor properties) :

.. code-block:: cpp

    #include <rtt/RTT.hpp>
    #include <rtt/TaskContext.hpp>
    #include <rtt/Component.hpp>
    #include <rtt/Logger.hpp>

    class MyComponent : public RTT::TaskContext
    {
       // Constructor
       // That's the name you're gonna pass as first argument of "loadComponent"
       MyComponent(const std::string& name):
       RTT::TaskContext(name)
       {
            RTT::log(RTT::Info) << "Constructing ! " << RTT::endlog();
       }

       // The function called when writing my_component.configure()
       void configureHook()
       {
            RTT::log(RTT::Info) << "Configuring  ! " << RTT::endlog();
       }

       // The function called (periodically or not) when calling my_component.start()
       void updateHook()
       {
            RTT::log(RTT::Info) << "Updating ! " << RTT::endlog();
       }
    };
    ORO_CREATE_COMPONENT(MyComponent) //Let Orocos know how to build this component


The ``CmakeLists.txt`` can look like this :

.. code-block:: cmake

    cmake_minimum_required(VERSION 2.8.3)
    project(my_component)

    find_package(catkin REQUIRED COMPONENTS
        # This will automatically import all Orocos components in package.xml,
        # and put them in ${USE_OROCOS_LIBRARIES}
        rtt_ros
        cmake_modules
    )

    include_directories(
        #include
        ${USE_OROCOS_INCLUDE_DIRS}
        ${CATKIN_INCLUDE_DIRS}
    )

    orocos_component(my_component MyComponent.cpp)
    set_property(TARGET my_component APPEND
            PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

    target_link_libraries(my_component
        ${USE_OROCOS_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    # orocos_install_headers(DIRECTORY include/${PROJECT_NAME})
    orocos_generate_package(INCLUDE_DIRS include)

Then you can just call ``cd my_component; mkdir build ; cd build ; cmake .. && make``. This will generate in the build directory what you can expect from a ROS package : a **devel/** directory containing all the targets (here "my_component") and a **setup.bash**.

.. note:: Using a `catkin workspace <http://wiki.ros.org/catkin/Tutorials/create_a_workspace/>`_ makes life much easier : you can put all your packages in ``src/``, build them all at once, and you'll have the ``setup.bash`` at ``my_ws/devel/setup.bash``


Now if you ``source devel/setup.bash`` and then call ``deployer`` , Orocos will know MyComponent in its environnement :

.. code-block:: ruby

    getComponentTypes() # You will see MyComponent !

    loadComponent("my_component","MyComponent")
    my_component.configure()
    my_component.start()


Using ``rtt_ros_integration`` you can also call :

.. code-block:: ruby

    import("rtt_rospack")
    ros.find("my_component")

Orocos documentation for building components : http://www.orocos.org/wiki/orocos/toolchain/getting-started/cmake-and-building

Orocos/ROS documentation for building components easily with catkin : https://github.com/orocos/rtt_ros_integration

.. _Orocos Builder's Manual: http://www.orocos.org/stable/documentation/rtt/v2.x/doc-xml/orocos-components-manual.html
