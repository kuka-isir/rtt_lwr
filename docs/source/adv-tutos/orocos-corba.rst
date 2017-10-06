OROCOS with CORBA and MQUEUE
----------------------------

In order to use the corba interface (connect multiple deployers together), you'll need to build the ``orocos_ws`` and ``rtt_ros_ws`` with :

.. code-block:: bash

    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_MQ=ON -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB

Reference : http://www.orocos.org/stable/documentation/rtt/v2.x/doc-xml/orocos-components-manual.html#orocos-corba
