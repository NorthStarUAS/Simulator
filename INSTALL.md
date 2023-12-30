# Installation and Notes

## Prerequisites (these need review)

    $ pip install opencv-python --user
    $ pip install panda3d --user

    # note: this can be installed straight from pip + git with a single magic command line I'll need to look up
    $ git clone https://github.com/NavPy/NavPy.git
    $ cd NavPy
    $ python setup.py install --user

    # no longer needed because 'transformations' is no longer needed
    $ pip install build --upgrade --user

    # this is no longer used anywhere in the code (so not needed)
    $ git clone https://github.com/NorthStarUAS/transformations.git
    $ cd transformations
    $ python -m build
    $ pip install dist/transformations-2013.6.29-py3-none-any.whl --user

## Python 3.12 note

    rdb writes:

    I’ve finished making experimental builds for Python 3.12, if you’re willing to
    try them out, just grab a link to the whl for your platform (and tagged
    “cp312”) and feed it to pip install:
    https://buildbot.panda3d.org/downloads/26fae21cb9ffed023a3dda385b88ebe213c5c200/

## Running in the Simulator Environment

    $ python.exe visuals.py
