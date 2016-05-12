##################################
Make you life easier with Kdevelop
##################################

Kdevelop is a nice IDE that supports cmake natively.
To install it, just type ``sudo apt-get install kdevelop``, then in a **terminal**, type ``kdevelop``.

.. tip::

        To enable sourcing the bashrc from the Ubuntu toolbar, ``sudo nano /usr/share/applications/kde4/kdevelop.desktop``
        and replace ``Exec=kdevelop %u`` by ``Exec=bash -i -c kdevelop %u``.



Import a catkin/CMake project
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Click on ``Project --> Open/Import Project...``

.. image:: /_static/kdev-import.png

Select the ``CMakeLists.txt`` inside your project.

.. image:: /_static/kdev-import-cmake.png


Select **you project** as the root directory.

.. image:: /_static/kdev-import-root.png

Correct the ``Build Directory`` if necessary, and if you've already built with ``catkin build``, you should see ``Using an already created build directory``.

.. image:: /_static/kdev-import-end.png

.. warning::

        Make sure the ``Build Directory`` is set to ``/path/to/catkin_ws/build/my_project``.

Click on ``finish`` and you're done import your project.

.. image:: /_static/kdev-import-finish.png



Build your project
~~~~~~~~~~~~~~~~~~

On the vertical left panel, click on ``Projects`` and you'll see the list of your currently opened projects. Yours should appear here afer a few seconds.

.. image:: /_static/kdev-editor.png

You can check also in the ``Build Sequence`` that your project appears.
To build, click on ``build`` :)

.. image:: /_static/kdev-build.png

.. note::

    Cliking on ``build`` is equivalent to calling :

    .. code::

        cd /path/to/catkin_ws
        catkin build my_super_project
