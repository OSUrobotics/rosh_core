#!/usr/bin/env python
import roslib; roslib.load_manifest('rosh')

# this changes the behavior of rospy/roslaunch/etc... on SIGINT to just be a normal KeyboardInterrupt
roslib.set_interactive(True)

import rosh
rosh.rosh_init()

# import all symbols after initialization
from rosh import *

import os, sys
plugins = sys.argv[1:]

for p in plugins:
    load(p, globals())

# load the user's roshrc file, if present
import roslib.rosenv as _rosenv
_roshrcp = os.path.join(_rosenv.get_ros_home(), 'rosh', 'roshrc.py')
if os.path.isfile(_roshrcp):
    print "loading roshrc"
    try:
        _f = open(_roshrcp)
        _plugins_copy = plugins[:]
        exec _f.read()
        if plugins and plugins != _plugins_copy:
            for p in plugins:
                try:
                    load(p, globals())
                except rosh.impl.exceptions.NoPlugin:
                    print >> sys.stderr, "ERROR: no plugin named [%s]"%(p)
                except rosh.impl.exceptions.InvalidPlugin, e:
                    print >> sys.stderr, "ERROR: plugin [%s] failed to load:\n\t%s"%(p, e)  

    finally:
        _f.close()
        del _f

del plugins
del sys
del os
del _roshrcp
del _rosenv
