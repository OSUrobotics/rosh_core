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
# Revision $Id: topic.py 13210 2011-02-14 01:05:08Z kwc $

"""
ROS topic API and implementation for ROSH
"""

from __future__ import with_statement


import os
import sys
import time
from collections import deque
from threading import Lock

import roslib.message
import roslib.names
from roslib.packages import get_pkg_dir

import rosmsg
import rostopic
import rospy

import rosh.impl.proc
from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept, NSResourceList

# key for adding plugin to show topic type
PLUGIN_SHOW_TOPIC_TYPE = 'show-topic-type'

LATEST = -1 # per API review, index of the buffered message

class _TopicIdxCallback(object):
    def __init__(self, idx):
        # per API review, topic[0] is really equal to the next
        # message, so start count at -1
        self.count = -1
        self.idx = idx + 1
        self.msg = None
        
    def __call__(self, msg):
        self.count += 1
        if self.count == self.idx:
            self.msg = msg

class _TopicSliceCallback(object):
    
    def __init__(self, ns_obj, start, stop, step, last_msg=None):
        if step == 0:
            raise ValueError("slice step cannot be zero")

        self.ns_obj = ns_obj

        # step_off is necessary for doing our step calculation
        # with the quirkiness of last_msg
        self.step_off = 0
        self.start = start
        self.stop = stop
        self.step = step or 1
        self.count = 0
        
        self.done = False
        self.started = False
        
        self.buff = deque()
        self.lock = Lock()

        # Python's behavior with out-of-bounds slices is to silently
        # succeed. This seems to mean that if last_msg is not
        # initialized and the start is 0, we ignore.
        if self.start == 0 or self.start == None:
            if last_msg is not None and self.stop > 0:
                self.started = True
                self.buff.append(last_msg)
                self.step_off = 1
            self.start = 1

        self.step_off = self.step_off - self.start
        if stop is not None and stop <= self.start:
            # note that we switch to the original start idx here to include last_msg
            self.done = True
        
    def __iter__(self):
        with subscribe(self.ns_obj, self):
            while not self.done:
                if self.buff:
                    # deques are threadsafe
                    yield self.buff.popleft()
                time.sleep(0.01) 
        # yield remainder
        while self.buff:
            yield self.buff.popleft()
        
    def __call__(self, msg):
        if not self.done:
            self.count += 1
            if self.count >= self.start:
                if not self.started or (self.count + self.step_off) % self.step == 0:
                    self.started = True
                    self.buff.append(msg)
            if self.stop is not None and self.count >= self.stop - 1:
                self.done = True
            
class subscribe(object):
    """
    Subscribe to topic instance.  'with:' semantics enable auto-unsubscribe. 
    """
    
    def __init__(self, ns_obj, cb=None):
        self.ns_obj = ns_obj
        if cb is not None:
            self.callbacks = [cb]
            self.ns_obj._add_subscriber_callback(cb)
        else:
            self.callbacks = []
            
    def __enter__(self):
        pass
    
    def __exit__(self, type, value, traceback):
        self.close()

    def add_callback(self, cb):
        """
        Register a new callback on this handle.
        
        Not threadsafe.
        """
        self.ns_obj._add_subscriber_callback(cb)
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        """
        Unregister callback on this handle.
        
        Not threadsafe.
        """
        self.ns_obj._remove_subscriber_callback(cb)
        self.callbacks.remove(cb)
    
    def close(self):
        for cb in self.callbacks:
            self.ns_obj._remove_subscriber_callback(cb)
        
        self.ns_obj = None
        self.cb = None

    def __repr__(self):
        if self.ns_obj is not None:
            return "subscribe[%s]: %s callback(s)"%(self.ns_obj._name, len(self.callbacks))
        else:
            return "<closed>"
            
def next_msg(ns_obj, timeout=None):
    """
    Get the next message on the topic

    @param ns_obj: TopicNS instance
    @type  ns_obj: L{TopicNS}
    @param timeout: timeout in millis
    @type  timeout: float
    @return: message
    @rtype: roslib.message.Message
    """
    cb = _TopicIdxCallback(1)
    
    if timeout is not None:
        timeout_t = time.time() + timeout
        with subscribe(ns_obj, cb):
            while cb.msg is None and time.time() < timeout_t:
                time.sleep(0.01) # yield
    else:
        with subscribe(ns_obj, cb):
            while cb.msg is None:
                time.sleep(0.01) # yield
    return cb.msg    

    
class TopicInfo(object):

    def __init__(self, config, name, pub_nodes, sub_nodes):
        self.name, self._pub_nodes, self._sub_nodes = name, pub_nodes, sub_nodes
        n = config.ctx.nodes
        node_config = n._config
        NodeNS = n._nstype
        self.pub_nodes = NSResourceList('', node_config, pub_nodes, NodeNS)
        self.sub_nodes = NSResourceList('', node_config, sub_nodes, NodeNS)

    def _update(self, pub_nodes, sub_nodes):
        """
        In-place update of resource lists
        """
        self._pub_nodes, _sub_nodes = pub_nodes, sub_nodes        
        self.pub_nodes._set_resources(pub_nodes)
        self.sub_nodes._set_resources(sub_nodes)

    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        buff ="Topic [%s]\n"%(self.name)
        if self._pub_nodes:
            buff += "\nPublishers:\n"
            buff += '\n'.join([" * %s"%(l) for l in self._pub_nodes]) + '\n' 
        if self._sub_nodes:
            buff += "\nSubscribers:\n"
            buff += '\n'.join([" * %s"%(l) for l in self._sub_nodes]) + '\n'                
        return buff
        
def topic_info(config, name):
    try:
        state = config.ctx.master.getSystemState()
        pubs, subs, _ = state
        subs = [x for x in subs if x[0] == name]
        if subs:
            subs = subs[0][1]
        pubs = [x for x in pubs if x[0] == name]
        if pubs:
            pubs = pubs[0][1]
    except:
        #TODO: log
        return None
    return TopicInfo(config, name, pubs, subs)
    
class TopicNS(Namespace):
    """
    L{TopicNS} provides lightweight accessors to a ROS topic and its data. 
    """
    
    def __init__(self, ns, config):
        """
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        super(TopicNS,self).__init__(ns, config)
        self._type_name = None
        self._type = None
        self._sub = None
        self._pub = None
        self._callbacks = []
        self._last_msg = None
        self._buffer = None
        self._bufferlen = None
        self._buffer_lock = None
        self._latch = True
        self._mux = None
        self._topic_info = None

    def _list(self):
        # TODO: add getTopics() to master?  This is not very
        # efficient. At the very least, need to start caching in
        # config.master.
        pubs, subs, _ = self._config.master.getSystemState()
        return set([x[0] for x in pubs if x[0].startswith(self._name)] + [x[0] for x in subs if x[0].startswith(self._name)])

    def _props(self):
        return ['buffer', 'latch']

    def _set_latch(self, val):
        if self._latch != val:
            self._latch = val
            # TODO: it would be good to abstract this in rospy better
            if self._pub is not None:
                self._pub.impl.is_latch = val
        
    def _set_buffer(self, val):
        """
        Initialize the buffer of this subscriber instance to the
        specified size. This call can have multiple side-effects:
        
         * it will start a subscription to the topic
         * if buffer is already initialized, this call will empty the buffer.

        @param val: length of subscriber callback buffer
        @type  val: int
        """
        self._bufferlen = val
        if val is not None:
            if self._buffer_lock is None:
                self._buffer_lock = Lock()
                
            # - unfortunately 'maxlen' is only supported in Python 2.6
            if self._buffer is not None:
                oldeque = self._buffer
                self._buffer = deque([])
                oldeque.clear()
            else:
                self._buffer = deque([])
        self._init_sub()

    def _info(self):
        if self._topic_info is None:
            self._topic_info = topic_info(self._config, self._name)
        return self._topic_info
    
    def _show(self):
        """
        show() API. Visualize this topic/namespace.
        """
        #TODO: make this pluggable so that visualizers can be attached to any type name
        if self._type is None:
            self._init_type()
        # any type_name checks must occur before type is None checks
        if self._type_name == 'nav_msgs/OccupancyGrid':
            if self._type is not None:
                show_occ_grid(self)
        elif self._type is None:
            # namespace
            show_graph(self)
        else:
            show_rostopic(self)
        return True
    
    def _cb(self, msg):
        """Message subscription callback"""
        self._last_msg = msg
        if self._buffer is not None:
            # have to lock due to potential to slice in separate thread
            b = self._buffer
            with self._buffer_lock:
                while len(b) > self._bufferlen - 1:
                    b.popleft()
                self._buffer.append(msg)

    def __repr__(self):
        return self.__str__()
    
    def __str__(self):
        if self._type_name is None:
            self._init_type()
        if self._type_name is None:
            return self._ns
        else:
            return rosmsg.get_msg_text(self._type_name)

    def _get_type(self):
        """
        rostype() API
        """
        if self._type is None:
            self._init_type()
        return self._type

    def _set_type(self, t):
        """
        rostype() API. Must be called with desired topic type. topic
        type can be a string with the message type name, a
        roslib.message.Message, or a Msg
        """
        if type(t) == str:
            t = roslib.message.get_message_class(t)
            if t is None:
                raise ValueError("cannot load message type: %s"%(t))
        else:
            if not type(t) == type:
                raise ValueError("invalid message type: %s"%(type(t)))
            if not issubclass(t, roslib.message.Message):
                raise ValueError("invalid message type: %s"%(t.__class__.__name__))

        # finish initalization
        with self._config.lock:
            if self._type is not None and t != self._type:
                self._cleanup()
            self._type = t
            self._type_name = t._type
        
    def _init_type(self):
        """
        Lazy-initialize type based on graph state.
        """
        if self._ns == '/':
            return
        if self._type is None:
            self._type_name = rostopic.get_topic_type(self._ns, blocking=False)[0]
            if self._type_name:
                self._type = roslib.message.get_message_class(self._type_name)
                if self._type is None:
                    pkg, base_type = roslib.names.package_resource_name(self._type_name)
                    print >> sys.stderr, "\ncannot retrieve type [%s].\nPlease type 'rosmake %s'"%(self._type_name, pkg)
    
    def _init_sub(self, block=False):
        """
        Lazy-init subscriber for topic
        """
        # This initialization is fairly fragile as it depends on the
        # type being introspectable.  I'll need to think of a way to
        # allow users to declare the type of a topic, but allow lazy
        # behavior as well.
        while self._sub is None:
            with self._config.lock:
                if self._sub is None:
                    #print "init_sub", self._ns
                    self._init_type()
                    if self._type is None:
                        if not block:
                            raise ROSHException("please init() this instance with the topic type")
                    else:
                        self._sub = rospy.Subscriber(self._ns, self._type, self._cb)
                        return
            if not block:
                return
            time.sleep(0.1)

    def _cleanup_pub(self):
        with self._config.lock:
            if self._pub is not None:
                self._pub.unregister()
                self._pub = None
        
    def _cleanup_sub(self):
        with self._config.lock:
            if self._sub is not None:
                self._sub.unregister()
                self._sub = None

    def _cleanup(self):
        with self._config.lock:
            # TODO: this should probably check to see if our the impl
            # is still present, and, if so, either tear down the impl
            # or error about type inconsistency.
            self._cleanup_pub()
            self._cleanup_sub()
        
    def __delattr__(self):
        self._cleanup()
                
    def _init_pub(self):
        """
        Lazy-init publisher for topic
        """
        # This initialization is fairly fragile as it depends on the
        # type being introspectable.  I'll need to think of a way to
        # allow users to declare the type of a topic, but allow lazy
        # behavior as well.
        if self._pub is None:
            with self._config.lock:
                if self._pub is None:
                    #print "init_pub", self._ns
                    self._init_type()
                    if self._type:
                        self._pub = rospy.Publisher(self._ns, self._type, latch=self._latch)
                    else:
                        raise ROSHException("please init() this instance with the topic type")            

    def _add_subscriber_callback(self, cb, cb_args=None):
        """
        Add callback to be invoked when new messages arrive
        """
        if self._sub is None:
            self._init_sub()
        self._sub.impl.add_callback(cb, cb_args)

    def _remove_subscriber_callback(self, cb, cb_args=None):
        self._sub.impl.remove_callback(cb, cb_args)        
    
    def __call__(self, *args, **kwds):
        """
        Publish with a topic-style namespace object. Implements call
        functionality for ROSNamespace objects when used with topics.

        @param args: *args
        @param kwds: **kwds
        """
        # lazy-init ns_obj
        if self._pub is None:
            # lazy-init type if user calls with a message class arg
            if len(args) == 1 and isinstance(args[0], roslib.message.Message):
                self._type = args[0].__class__
            self._init_pub()
        self._pub.publish(*args, **kwds)

    def _slice(self, slice_obj):
        self._init_sub(block=True)
        
        start, stop, step = slice_obj.start, slice_obj.stop, slice_obj.step
        step = step or 1
        if step < 1:
            raise ValueError("step size must be positive")

        if start is None or start >= 0:
            if stop is not None and start > stop:
                raise ValueError("stop index must be greater than end index")

            cb = _TopicSliceCallback(self, start, stop, step, self._last_msg)
            return cb
        else:

            # print to console
            if self._buffer is None:
                print >> sys.stderr, "This topic is not buffered. Use props(obj, buffer=N) to enable."
                return []

            # we don't allow this because there are race conditions that are hard to protect against
            if stop is not None and stop > 0:
                raise IndexError("cannot slice from past to future")
            
            # we interpret 0 to mean the last message, so, for the
            # purposes of a slice, it's the end of the history buffer
            if stop == 0:
                stop = None
                
            # we use python's interpretation of start being
            # out-of-bounds, which is to just silently succeed
            # - do this under lock as cb is happening in separate
            #   thread
            with self._buffer_lock:
                retval = list(self._buffer)[start:stop:step]
            return retval
        
    def _idx(self, idx):
        if idx > 0:
            # if user asks for future message, block until topic exists
            self._init_sub(block=True)
        else:
            self._init_sub(block=False)
            
        if idx < 0:
            if idx == LATEST: # -1
                # per API review, return buffered message
                return self._last_msg
            else:
                #TODO
                #check history buffer
                raise IndexError("[%s] outside of history buffer"%idx)
        else:
            cb = _TopicIdxCallback(idx)
            with subscribe(self, cb):
                while cb.msg is None:
                    time.sleep(0.01) # yield
            return cb.msg

    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)

        from rosh.impl.top_tools import Mux
        #TODO: allow assignment to string topic spec
        if not type(value) == TopicNS:
            raise ValueError("value must be a topic")

        output_ns = self.__getitem__(key)
        if output_ns._mux is None:
            # implement as mux so that it can be reconfigurable. With
            # a relay we have to start/stop a process each time
            # reassignment occurs.
            m = Mux(self._config.ctx, [value], output_ns)
            output_ns._mux = m
            m.start()
            return m
        else:
            output_ns._mux.switch(value)
            return output_ns._mux
    
    def __getitem__(self, key):
        """
        Dictionary-style accessor for services
        """
        # This used to allow fetching fields directly, but I think
        # that confuses the API too much as the user can't tell when
        # they've bottomed out in a topic and the semantics of which
        # message you're looking at are unclear.
        if type(key) == slice:
            return self._slice(key)
        elif type(key) == int:
            return self._idx(key)
        # - get namespace
        else:
            return super(TopicNS, self).__getitem__(key)

class Topics(Concept):
    
    def __init__(self, ctx, lock):
        super(Topics, self).__init__(ctx, lock, TopicNS)
        
    def _show(self):
        show_graph(self._root)

    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)
        else:
            return self._root.__setattr__(key, value)


def show_graph(ns_obj):
    #TODO: in ROS 1.3, rxgraph will be a node within rxgraph so we can use launch instead of run()
    success = rosh.impl.proc.launch(ns_obj._config.ctx, 'rxgraph', 'rxgraph', args=['--topicns', ns_obj._ns])
    if success:
        print "showing graph of topics in namespace %s"%ns_obj._ns
    else:
        print >> sys.stderr, "failed to launch rxgraph"
    
def show_rostopic(ns_obj):
    # rostopic echo in a separate window
    ros_root = roslib.rosenv.get_ros_root()
    success = rosh.impl.proc.run(ns_obj._config, ['xterm', '-e', os.path.join(ros_root, 'bin', 'rostopic'), 'echo', ns_obj._name], stdout=False)
    # TODO: return proc object instead
    if success:
        print "running rostopic echo in a separate window"
    else:
        print >> sys.stderr, "unable to run rostopic echo in a separate window"
