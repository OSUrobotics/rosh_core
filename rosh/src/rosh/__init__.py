#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: __init__.py 11676 2010-10-22 02:06:13Z kwc $
"""
ROS Shell (rosh).

Higher-level, interactive scripting environment for ROS.
"""

from __future__ import with_statement
import roslib; roslib.load_manifest('rosh')

import os
import sys

import roslib.packages

# declare shell globals

# These are initialized by rosh.impl.ros_graph but declared here due
# to get_*() accessors
nodes = topics = services = parameters = msg = srv = None

# - by default, all publishers latch
latching = True

import rosh.plugin
import rosh.impl.ros_packages
import rosh.impl.ros_graph
import rosh.impl.ipy
import rosh.impl.namespace

from rosh.impl.namespace import rostype, info
from rosh.impl.show import show
from rosh.impl.props import props

# NOTE: this shadows the rosparam module. Is this an issue?
from rosh.impl.param import rosparam, rosparam_str

# client symbols
from rosh.impl.exceptions import ROSHException
from rosh.impl.library import findros
from rosh.impl.bagy import Bagy, MapBagy
from rosh.impl.bag import Bag

from rosh.impl.service import Service
from roslaunch.core import Node

def kill(obj):
    if hasattr(obj, '_kill'):
        obj._kill()
    elif type(obj) in (list, tuple):
        for x in obj:
            kill(x)
    else:
        raise ValueError("Not a kill-able object")

def load(name, globals_=None):
    """
    Load plugin. This loads the plugin into the rosh global symbol
    table. If globals_ is specified, plugin symbols will also be
    loaded into the provided dictionary. Common usage is::
    
        load_plugin('plugin_name', globals())

    @param globals_: global symbol table to additionally load plugin to.
    @type  globals_: dict
    """
    rosh.plugin.load_plugin(name, _plugin_context, globals_)
        
_plugin_context = None
def get_default_plugin_context():
    return _plugin_context

def ok():
    """
    @return: True if ok to keep executing, False if script should exit.
    """
    return rosh.impl.ros_graph.ok()

def rosh_init():
    # context is the heart of the rosh namespace logic
    global _ctx, _rosh_lock

    _ctx = rosh.impl.namespace.Context()

    import threading
    _rosh_lock = threading.RLock()    

    # plugin context is the loading mechanism for ROSH plugins
    global _plugin_context
    _plugin_context = rosh.plugin.PluginContext(globals(), _ctx, _rosh_lock)

    # load symbols for ROS packages layer
    rosh.impl.ros_packages.load_rosh_plugin('rosh.impl.ros_packages', _plugin_context)
    
    # load symbols for ROS graph layer. Note: graph layer and package
    # layer are coupled by both msg/srv and roslaunch, which makes
    # separating rosh less than useful right now.
    rosh.impl.ros_graph.load_rosh_plugin('rosh.impl.ros_graph', _plugin_context)

    # initialize IPython Magic
    rosh.impl.ipy.ipy_magic_init()

# initialize privates

# lock for member/global initialization
_rosh_lock = None
# ROSH shared context
ctx = None

