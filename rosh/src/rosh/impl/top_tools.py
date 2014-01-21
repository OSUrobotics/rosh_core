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
# Revision $Id: top_tools.py 11681 2010-10-22 07:08:51Z kwc $

from rosh.impl.exceptions import ROSHException
import rosh.impl.proc

#TODO (long term): add remote launch capability, as that's where topic
#tools are most useful.

class TopicTool(object):
    """
    Abstraction of general topic tool API
    """
    
    def __init__(self, ctx, type_, ns_obj_input, ns_obj_output, name=None):
        if ns_obj_input._name == ns_obj_output._name:
            raise ValueError("cannot %s to same topic"%(type))

        self._ctx = ctx
        self.type = type_
        self.topic_input = ns_obj_input
        self.topic_output = ns_obj_output
        if name is None:
            self.name = '/%s%s'%(self.type, self.topic_output._name)
        else:
            self.name = name
        
        self._proc = self._node = None

    def _start(self, args):
        if self._proc is not None:
            raise ROSHException("Already started")
        self._node, self._proc = \
            rosh.impl.proc.launch_node(self._ctx, 'topic_tools', self.type, args=args)

    def kill(self):
        # kill() API require _kill()
        self._kill()
        
    def _kill(self):
        if self._proc is not None:
            self._proc._kill()
            self._proc = None
            
            self._node = None
        else:
            print >> sys.stderr, "nothing to kill"

class Mux(TopicTool):
    """
    Interface to topic_tools/mux
    """
    
    def __init__(self, ctx, ns_obj_inputs, ns_obj_output, name=None):
        if not ns_obj_inputs or not type(ns_obj_inputs) in (list, tuple):
            raise ValueError("ns_obj_inputs must be a non-empty list of input topics")
        TopicTool.__init__(self, ctx, 'mux', ns_obj_inputs[0], ns_obj_output, name=name)
        self.initial_topic_inputs = ns_obj_inputs
        self._history = [x._name for x in ns_obj_inputs]        

    def start(self):
        args = [self.topic_output._name] + [x._name for x in self.initial_topic_inputs] + \
            ["mux:=%s"%self.name, "__name:=%s"%self.name]
        self._start(args)

    def __call__(self, input_ns):
        return self.switch(input_ns)

    def switch(self, input_ns):
        if input_ns._name not in self._history:
            self._ctx.services[self.name].add(input_ns._name)
            self._history.append(input_ns._name)
        res = self._ctx.services[self.name].select(input_ns._name)
        self.topic_input = input_ns
        return res
        
class Relay(TopicTool):
    """
    Interface to topic_tools/relay
    """
    
    def __init__(self, ctx, ns_obj_input, ns_obj_output, name=None, unreliable=False):
        TopicTool.__init__(self, ctx, 'relay', ns_obj_input, ns_obj_output, name=name)
        self.unreliable = unreliable

    def start(self):
        param = 'true' if self.unreliable else 'false'
        args = [self.topic_input._name, self.topic_output._name, 
                "__name:=%s"%self.name, "_unreliable:=%s"%param]
        self._start(args)
    
THROTTLE_MODE_MESSAGES = 'messages'
THROTTLE_MODE_BYTES = 'bytes'
class Throttle(TopicTool):
    """
    Interface to topic_tools/relay
    """
    
    def __init__(self, ctx, mode, ns_obj_input, ns_obj_output, rate, name=None, window=None):
        if mode == THROTTLE_MODE_MESSAGES:
            pass
        elif mode == THROTTLE_MODE_BYTES:
            if window is None:
                raise ValueError("window must be specified")
        else:
            raise ValueError("invalid throttle mode [%s]"%(mode))

        TopicTool.__init__(self, ctx, 'throttle', ns_obj_input, ns_obj_output, name=name)
        self.mode = mode
        self.rate = rate
        self.window = window

    def start(self):
        params = [str(self.rate)]
        if self.mode == THROTTLE_MODE_BYTES:
            params.append(str(self.window))
            
        args = [self.mode, self.topic_input._name] + params + [self.topic_output._name, "__name:=%s"%self.name]
        self._start(args)

################################################################################
# Factories: these functions create function handles with the context
# already initialized so that users don't have to retrieve the plugin
# context.

def mux_factory(ctx):
    def mux(intopic_objs, outtopic_obj, name=None):
        """
        Create a Mux. The mux will start off selected to the first of the in topics.

        @param intopic_objs: list of input topics
        @type  intopic_objs: [TopicNS]
        @param outtopic_obj: output topic
        @type  outtopic_obj: [TopicNS]
        @return: running Mux
        @rtype: Mux
        """
        m = Mux(ctx, intopic_objs, outtopic_obj, name=name)
        m.start()
        return m
    return mux

def throttle_factory(ctx):
    def throttle(intopic_obj, outtopic_obj, msgs_per_sec, name=None):
        """
        Create throttled topic controlled by message rate.

        @param intopic_obj: input topic
        @type  intopic_obj: [TopicNS]
        @param outtopic_obj: output topic
        @type  outtopic_obj: [TopicNS]
        @param msgs_per_sec: Messages per second to let through
        @type  msgs_per_sec: int

        @return: running Throttle
        @rtype: Throttle
        """
        t = Throttle(ctx, THROTTLE_MODE_MESSAGES, intopic_obj, outtopic_obj, msgs_per_sec, name=name)
        t.start()
        return t
    return throttle

def throttlebw_factory(ctx):
    def throttlebw(intopic_obj, outtopic_obj, bytes_per_sec, window=1.0, name=None):
        """
        Create throttled topic controlled by max bandwidth.

        @param intopic_obj: input topic
        @type  intopic_obj: [TopicNS]
        @param outtopic_obj: output topic
        @type  outtopic_obj: [TopicNS]
        @param bytes_per_sec: Maximum bytes per second to let through
        @type  bytes_per_sec: int
        @param window: (optional) window over which to sample bandwidth, default 1.0.
        @type  window: float

        @return: running Throttle
        @rtype: Throttle
        """
        t = Throttle(ctx, THROTTLE_MODE_BYTES, intopic_obj, outtopic_obj, bytes_per_sec, name=name, window=window)
        t.start()
        return t
    return throttlebw

def relay_factory(ctx):
    def relay(intopic_obj, outtopic_obj, unreliable=False, name=None):
        """
        Create a relay. This is useful where
        latency is more important that receiving all packets,
        e.g. operating over a wireless network.

        @param intopic_obj: input topic
        @type  intopic_obj: [TopicNS]
        @param outtopic_obj: output topic
        @type  outtopic_obj: [TopicNS]
        @param unreliable: if True, uses unreliable (UDP)
        transport. default False
        @type  unreliable: bool

        @return: running Relay
        @rtype: Relay
        """
        r = Relay(ctx, intopic_obj, outtopic_obj, name=name, unreliable=unreliable)
        r.start()
        return r
    return relay

def udprelay_factory(ctx):
    def udprelay(intopic_obj, outtopic_obj, name=None):
        """
        Create a relay with unreliable transport. This is useful where
        latency is more important that receiving all packets,
        e.g. operating over a wireless network.

        @param intopic_obj: input topic
        @type  intopic_obj: [TopicNS]
        @param outtopic_obj: output topic
        @type  outtopic_obj: [TopicNS]

        @return: running Relay
        @rtype: Relay
        """
        r = Relay(ctx, intopic_obj, outtopic_obj, name=name, unreliable=False)
        r.start()
        return r
    return udprelay
          
def topic_tools_symbols(ctx):
    """
    Generate API for topic tools based on ctx instance.
    
    @return: dictionary of symbols for topic tools user-facing API
    """
    return {
        'relay': relay_factory(ctx),
        'udprelay': udprelay_factory(ctx),
        'mux': mux_factory(ctx),
        'throttle': throttle_factory(ctx),
        'throttlebw': throttlebw_factory(ctx)
        }
    
