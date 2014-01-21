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
# Revision $Id: ros_graph.py 13210 2011-02-14 01:05:08Z kwc $

import rosgraph.masterapi

import rosh.plugin
from rosh.impl.launch import launch_symbols
from rosh.impl.namespace import Namespace
from rosh.impl.node import Nodes
from rosh.impl.param import Parameters
from rosh.impl.service import Services
from rosh.impl.topic import Topics, LATEST, subscribe
from rosh.impl.top_tools import topic_tools_symbols

from rosh.impl.msg import Msgs
from rosh.impl.msg import Srvs

import rospy

# initialize globals
def init_master(plugin_context):
    master = rosgraph.masterapi.Master('rosh')
    plugin_context.ctx.master = master
    return master.is_online()

def serve_forever():
    rospy.spin()

def ok():
    return not rospy.is_shutdown()

def wait_for_service(service, timeout=None):
    """
    Equivalent to rospy.wait_for_service(), but also handles unboxing of ROSH objects..

    @param service: Service
    @type  service: Namespace or str
    @param timeout: (optional) wait_for_service timeout
    @type  timeout: float
    """
    if isinstance(service, Namespace):
        rospy.wait_for_service(service._name, timeout=timeout)  
    else:
        rospy.wait_for_service(service, timeout=timeout)

# we use the general plugin loading framework to initialize to keep things tidy
_loaded_symbols = None
def load_rosh_plugin(name, plugin_context, globals_=None):
    global _loaded_symbols
    if rosh.plugin.reentrant_load(_loaded_symbols, globals_):
        return

    ctx = plugin_context.ctx
    rosh_lock = plugin_context.rosh_lock

    if init_master(plugin_context):
        rospy.init_node('rosh', anonymous=True, disable_signals=True)

        _loaded_symbols = {
            # useful aliases
            'now': rospy.get_rostime,
            'Header': rospy.Header,
            'Time': rospy.Time,
            'Duration': rospy.Duration,
            'Rate': rospy.Rate,
            'sleep': rospy.sleep,
            'logfatal': rospy.logfatal,
            'logerr': rospy.logerr,
            'loginfo': rospy.loginfo,                        
            'logdebug': rospy.logdebug,

            # rospy by default (per API review)
            'rospy': rospy,

            #concepts
            'nodes': Nodes(ctx, rosh_lock),
            'topics': Topics(ctx, rosh_lock),
            'services': Services(ctx, rosh_lock),
            'parameters': Parameters(ctx, rosh_lock),
            'srv': Srvs(ctx, rosh_lock),
            'msg': Msgs(ctx, rosh_lock),
            'serve_forever': serve_forever,

            #constants
            'LATEST': LATEST,

            # api
            'wait_for_service': wait_for_service,            
            'subscribe': subscribe,
            }
        # We use the loader style below for modules that export
        # numerous symbols to let them manage which symbols they are
        # exporting directly.
        
        # Load topic tools
        _loaded_symbols.update(topic_tools_symbols(ctx))
        # Load roslaunch
        _loaded_symbols.update(launch_symbols(ctx))
        
        # save these symbols in the context
        ctx.msg = _loaded_symbols['msg']
        ctx.srv = _loaded_symbols['srv']

        ctx.nodes      = _loaded_symbols['nodes']
        ctx.topics     = _loaded_symbols['topics']
        ctx.services   = _loaded_symbols['services']
        ctx.parameters = _loaded_symbols['parameters']
        rosh.plugin.globals_load(plugin_context, globals_, _loaded_symbols)

