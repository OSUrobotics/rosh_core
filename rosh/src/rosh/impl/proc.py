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
# Revision $Id: proc.py 11427 2010-10-06 23:39:08Z kwc $

"""
ROSH library for running processes. 

This library is necessary as ROSH commands, like set_master, can
manipulate the environment in ways that would affect subprocesses.
"""

import os
from subprocess import Popen, PIPE

import roslib.rosenv
import roslaunch

# TODO: get rid of this routine or replace with Exec
def run(config, cmd, stdout=True):
    """
    Run the specified command using the current ROS configuration.
    """
    env = os.environ.copy()
    env[roslib.rosenv.ROS_MASTER_URI] = config.master.master_uri
    
    if 0:
        print "CMD", cmd

    # TODO: added Exec objects to roslaunch that are only accessible via scriptapi
    if stdout:
        p = Popen(cmd, stderr=PIPE, env=env)
    else:
        p = Popen(cmd, stdout=PIPE, stderr=PIPE, env=env)
    
    p.poll()
    return p.returncode in [None, 0]

def _init_ctx(ctx):
    """
    Add shared roslaunch instance to ctx if not already initialized.
    
    @param ctx: ROSH context
    @type  ctx: Contextj
    """
    if not ctx._roslaunch.started:
        ctx._roslaunch.start()
    
#TODO: __str__ debugging info
class Process(object):
    """
    ROSH wrapper for processes to make them kill-able
    """
    
    def __init__(self, roslaunch_process):
        self.roslaunch_process = roslaunch_process
        
    def _kill(self):
        self.roslaunch_process.stop()
    
def launch_node(ctx, pkg, type_, args=[], remap={}, stdout=False):
    """
    Low-level launch() API for internal ROSH and ROSH plugin use.
    
    @param ctx: ROSH context
    @type  ctx: Context
    @param pkg: package name
    @param type_: node type
    @return: Node and Process instance
    @rtype: (roslaunch.Node, roslaunch.Process)
    """
    output = 'screen' if stdout else None
    if type(args) in (list, tuple):
        args = ' '.join([str(x) for x in args])
    if remap:
        args = args + ' '.join(['%s:=%s'%(k, v) for k, v in remap.iteritems()])

    return launch_roslaunch_Node(ctx, roslaunch.Node(pkg, type_, args=args, output=output, filename='<rosh>'))

def launch_roslaunch_file(ctx, roslaunch_filename):
    # TODO: use roshlaunch API
    raise NotImplemented

def launch_roslaunch_Node(ctx, node):
    """
    Launch a Node instance. This is a low-level launch() API for
    internal ROSH and ROSH plugin use.
    
    @param ctx: ROSH context
    @type  ctx: Context
    @param node: node instance. The name attribute may be overwritten.
    @type  node: roslaunch.Node
    @return: Node instance
    @rtype: Node
    """
    _init_ctx(ctx)
    p = ctx._roslaunch.launch(node)
    return node, Process(p)    
