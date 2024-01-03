# Installation and Notes

## Prerequisites (these need review)

Quick note: there are many ways to install python packages, your distribution
(i.e. conda or ubuntu or fedora) may have other preferred ways to install
packages.  You may have some differnet preferred approach.  This is all totally
fine, do whatever makes sense for you, but this is one way to install the
prerequisites.

    $ pip install opencv-python --user
    $ pip install panda3d --user
    $ pip install pygeotile --user

    # note: navpy can be installed straight from pip + git with a single magic command line I'll need to look up
    $ git clone https://github.com/NavPy/NavPy.git
    $ cd NavPy
    $ python setup.py install --user

## Python 3.12 note

    rdb writes:

    I’ve finished making experimental builds for Python 3.12, if you’re willing to
    try them out, just grab a link to the whl for your platform (and tagged
    “cp312”) and feed it to pip install:
    https://buildbot.panda3d.org/downloads/26fae21cb9ffed023a3dda385b88ebe213c5c200/

## Installing the nsWorld module

    $ python -m build
    $ pip install --user dist/nsworld-1.0-py3-none-any.whl

## Running demo visual system

    $ python.exe visuals.py
