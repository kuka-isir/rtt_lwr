Build the documentation
------------------------

Documentation is built using **readthedocs**, using itself **sphinx**, and hosted on https://readthedocs.org/.
Everytime a push is made on github, a new build is triggered at http://rtt-lwr.readthedocs.io/.

Syntaxe
~~~~~~~

It uses **reStructeredText** instead of default **markdown** in order to support math equations, notes, warnings etc.
More info at https://docs.readthedocs.io/en/latest/getting_started.html

Generate documentation locally
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    sudo -H pip install -U pip sphinx sphinx-autobuild recommonmark sphinx_rtd_theme

* Go to the docs directory : ``roscd rtt_lwr/../docs`` and type ``make livehtml``.
* Open your favorite webbrowser and go to ``127.0.0.1/8000`` to see the generated site locally.

Update the doc using your favorite text editor, like atom (https://atom.io), and `open a pull request <https://help.github.com/articles/about-pull-requests/>`_ on github at https://github.com/kuka-isir/rtt_lwr.
