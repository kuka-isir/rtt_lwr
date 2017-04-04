**Update the packages**
#########################

We are periodically doing updates on the code (new tools, bug fixes etc), so keeping it up-to-date can be very useful.

To update all the packages we're going to use the ``vcs-tools`` utility (https://github.com/dirk-thomas/vcstool).

It should be already installed during the :doc:`installation </install/install-16.04-kinetic>` procedure, otherwise ``sudo apt install python-vcstool``.

Update OROCOS
-------------

.. code-block:: bash

        cd ~/isir/orocos-2.9_ws/src
        vcs pull
        # update submodules for bleeding-edge updates (OPTIONAL)
        vcs-git submodule foreach git checkout toolchain-2.9
        vcs-git submodule foreach git pull

Update RTT ROS Integration
--------------------------

.. code-block:: bash

        cd ~/isir/rtt_ros-2.9/src
        vcs pull

Update rtt_lwr
--------------

.. code-block:: bash

        cd ~/isir/lwr_ws/src
        vcs pull
        vcs-git submodule update

Update the documentation
------------------------

Locally using sphinx
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

        sudo -E pip install sphinx sphinx-autobuild recommonmark sphinx_rtd_theme


If you're **not** part of the rtt_lwr developers :

* Create an account on ``github``
* Go to `rtt_lwr repo <https://github.com/kuka-isir/rtt_lwr>`_ and fork it !
* Clone it on your computer : ``git clone https://github.com/mysuperaccountname/rtt_lwr``.

Once you have a recent copy of rtt_lwr :

* Go to the docs directory : ``roscd rtt_lwr/../docs`` and type ``make livehtml``.
* Open your favorite webbrowser and go to ``127.0.0.1/8000`` to see the generated site locally.

Update the doc using your favorite text editor, like atom (https://atom.io).

One updated, ``git add -A`` and ``git commit -m "my super contribution"`` and `open a pull request <https://help.github.com/articles/using-pull-requests/>`_, that will be merged once validated.

Directly on github
~~~~~~~~~~~~~~~~~~

Simply go to https://github.com/kuka-isir/rtt_lwr and edit files in ``docs``. The only drawback is that you can only edit one file at a time.
