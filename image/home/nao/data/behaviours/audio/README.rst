To run on OSX or Ubuntu:

1.  [Optional] Install pip & virtualenv
    (especially if you are likely to work on more than one python project,
     it helps manage specific project requirements which helps avoid
     "works for me" but breaks for everyone else).

    *   Install pip:

        https://pip.pypa.io/en/latest/installing.html

    *   Installing ``virtualenv`` and ``virtualenvwrapper``::

            pip install virtualenv
            pip install virtualenvwrapper

    *   Add this to your ``~/.profile``, assuming you keep your projects in
        ``~/Projects``::

            # Setup virtualenvwrapper.sh for Python virtualenv
            export WORKON_HOME=$HOME/.virtualenvs
            export PROJECT_HOME=$HOME/Projects
            source /usr/local/bin/virtualenvwrapper.sh

    *   Reload your Terminal or ``source ~/.profile``

    *   Setup a virtualenv for our project::

            mkvirtualenv rUNSWift

    *   Add to the post-activate hook::

            vi ~/.virtualenvs/rUNSWift/bin/postactivate

        i.e. add the following::

            # Also switch directory when starting this virtualenv
            cd ~/Projects/rUNSWift/

    **Check**: In a new Terminal window, ``workon rUNSWift``
    adds ``(rUNSWift)`` to the start of your prompt,
    and changes directory for you.


2.  Install requirements:

    cd ${RUNSWIFT_CHECKOUT_DIR}/image/home/nao/data/behaviours/audio
    pip install -r requirements.txt


3.  Run regression test suite:

    cd ${RUNSWIFT_CHECKOUT_DIR}/image/home/nao/data/behaviours/audio
    python whistle_detector.py

    **Check**: Should print a bunch of test results, something like::

         --------------------------------------------------------------------------------
         TEST RESULTS: run=9, success=9
         --------------------------------------------------------------------------------

