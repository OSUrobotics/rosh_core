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
# Revision $Id: launch.py 11427 2010-10-06 23:39:08Z kwc $

"""
roslaunch API for clients. The lower-level implementation is in
rosh.impl.proc, which is the general process system of
rosh. rosh.impl.launch is just the roslaunch-level API for clients.
"""

import os
import sys

import roslib.packages
import roslaunch

import rosh.impl.proc

def launch_factory(ctx):

    def launch(launchable, type_=None, args=[], remap={}):
        """
        Launch things that are launchable. This includes::

          launch('path/to/file.launch')
          launch('my_pkg', 'file.launch')
          
          launch('path/to/node')
          launch('pkg', 'nodetype')
          launch('pkg')  # == launch('pkg', 'pkg'), e.g. 'rviz'
          
          launch(roslaunch.Node)
          
        @param launchable: ROS package name or Node instance
        @type  launchable: str or Node
        @param type_: node type (executable name). If None and
            launchable is a package name, this will default to package
            name.
        @type  type_: str
        @param args: node arguments
        @type  args: [str]
        @param remap: remapping arguments
        @type  remap: dict

        @return: List of nodes launched
        @rtype: [rosh.impl.Node]
        """
        if type(launchable) == str:

            if os.path.isfile(launchable):
                if launchable.endswith('.launch'):
                    if type_:
                        raise ValueError("launch of launch file does not accept additional type_ arg")
                    if remap:
                        raise ValueError("launch of launch file does not accept remap arguments")
                    
                    return _launch_roslaunch_file(ctx, launchable, args)
                else:
                    # reverse-derive package name
                    pkg, pkg_dir = roslib.packages.get_dir_pkg(launchable)
                    if pkg == None:
                        raise ValueError("launch: cannot determine package of [%s]"%(launchable))
                    type_ = os.path.basename(launchable)
                    return _launch_node(ctx, pkg, type_, args, remap)                
            else:
                # validate launchable is a package name
                try:
                    roslib.packages.get_pkg_dir(launchable)
                except roslib.packages.InvalidROSPkgException:
                    raise ValueError("launch: [%s] does not appear to be a package or filename"%(launchable))

                # launch('rviz') -> launch('rviz', 'rviz')
                if type_ is None:
                    type_ = launchable

                # determine if we are launching a file or a node
                if type_.endswith('.launch'):
                    if remap:
                        raise ValueError("launch of launch file does not accept remap arguments")
                    return _launch_roslaunch_file(ctx, findros(launchable, type_), args)
                else:
                    return _launch_node(ctx, launchable, type_, args, remap)
            
        elif type(launchable) == roslaunch.Node:
            if type_:
                raise ValueError("launch of Node instance does not accept additional type_ arg")
            return _launch_Node(ctx, launchable, args, remap)
        elif hasattr(launchable, '_launch'):
            return launchable._launch(*args, **remap)

    return launch

def _launch_roslaunch_file(ctx, filename, args):
    val = rosh.impl.proc.launch_roslaunch_file(ctx, pkg, type_, args)
    return [_node_ref(ctx, n, p) for n, p in val]
    
def _launch_Node(ctx, n, args, remap):
    if args:
        if type(args) in (list, tuple):
            args = ' '.join([str(x) for x in args])
        n.args = args
    if remap:
        raise NotImplemented
    n, p = rosh.impl.proc.launch_roslaunch_Node(ctx, n)
    return [_node_ref(ctx, n, p)]
    
def _launch_node(ctx, pkg, type_, args, remap):
    n, p = rosh.impl.proc.launch_node(ctx, pkg, type_, args, remap=remap) 
    return [_node_ref(ctx, n, p)]

def _node_ref(ctx, node, process):
    """
    Convert to rosh.impl.Node reference and attach process for killing
    @param node: roslaunch Node instance. 'name' attribute must be initialized
    @type  node: roslaunch.Node
    @param process: Process instance
    @type  process: rosh.impl.proc.Process
    """
    
    # TODO: split apart namespaces
    ns_node = ctx.nodes[node.name]
    # attach process for _kill
    ns_node._process = process
    return ns_node
    
def rosrun_factory(ctx):
    def rosrun(pkg, type_, args=[]):
        n, p = rosh.impl.proc.launch(ctx, pkg, type_, args)
        ns_node = _node_ref(ctx, n)
        # attach process for _kill
        ns_node._process = p
        return ns_node
    return rosrun

def launch_symbols(ctx):
    # store shared roslaunch instance in context. All Launchable items
    # need access to this.

    # _roslaunch is the underlying roslaunc instance for this
    # ctx. ctx.launch is the plugin-accessible API.
    ctx._roslaunch = roslaunch.ROSLaunch()
    ctx.launch = launch_factory(ctx)
    return {
        'rosrun': rosrun_factory(ctx),
        'launch': ctx.launch,
        }
