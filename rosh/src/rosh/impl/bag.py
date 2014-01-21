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
# Revision $Id: bagy.py 11435 2010-10-07 18:03:49Z kwc $

from __future__ import with_statement

import threading

import roslib.message
import rosbag
        
import rosh
from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Context, NamespaceConfig, Namespace
from rosh.impl.topic import TopicNS

class BagTopic(Namespace):

    def __init__(self, name, config):
        super(BagTopic, self).__init__(name, config)
        self._type = None
        
    def __iter__(self):
        return self._config.bag.read_messages(topics=[self._name])

    def _list(self):
        return self._config.connections.iterkeys()

    def _get_type(self):
        # rostype API
        if self._type is None:
            try:
                self._type = roslib.message.get_message_class(self._config.connections[self._name].datatype)
            except KeyError:
                raise TypeError("[%s] does not have a rostype"%(self._name))
        return self._type

# NamespaceConfig infrastructure requires a lock
_lock = threading.RLock()

ZERO_STATE = 0
RECORD_STATE = 1
STOP_STATE = 2

class Bag(rosbag.Bag):
    
    def __init__(self, *args, **kwds):
        """
        Constructor is the same as rosbag.Bag, with the addition of the 'ctx' keyword argument.

        @param ctx: ROSH Context
        @type  ctx: Context
        """
        if 'ctx' in kwds:
            self._ctx = kwds['ctx']
            del kwds['ctx']
        else:
            self._ctx = rosh.get_default_plugin_context().ctx
        super(Bag, self).__init__(*args, **kwds)
        self._lock = threading.Lock()
        self._record_topics = None
        self._state = ZERO_STATE

    def __iter__(self):
        return self.read_messages()
    
    def _get_topics(self):
        d = {}
        for c in self._get_connections():
            d[c.topic] = c

        config = NamespaceConfig(self._ctx, _lock)
        config.connections = d
        config.bag = self
        return BagTopic('', config)
 
    topics = property(_get_topics)

    def _sub_callback(self, msg, topic_name):
        with self._lock:
            # should this check for header?
            self.write(topic_name, msg, rosh.now())

    def record(self, topic):
        if isinstance(topic, TopicNS):
            pass
        elif type(topic) == str:
            topic = self._ctx.topics[topic]

        if self._record_topics is None:
            self._record_topics = []

        # start recording if we haven't been explicitly stopped()
        self._record_topics.append(topic)
        
        if self._state == ZERO_STATE:
            self._state = RECORD_STATE

        if self._state == RECORD_STATE:
            # TODO: clean this up once I add proper subscribe() to topic
            topic._add_subscriber_callback(self._sub_callback, topic._name)

    def close(self):
        if self._state == RECORD_STATE:
            self.stop()
            self._state = ZERO_STATE
        del self._record_topics
        super(Bag, self).close()
        
    def start(self):
        if self._record_topics is None or self._state == ZERO_STATE:
            raise ROSHException("not recording")
        if self._state == STOP_STATE:
            for topic in self._record_topics:
                topic._add_subscriber_callback(self._sub_callback, topic._name)
            self._state = RECORD_STATE
            
    def stop(self):
        if self._record_topics is None or self._state == ZERO_STATE:
            raise ROSHException("not recording")
        
        if self._state == RECORD_STATE:
            for topic in self._record_topics:
                topic._remove_subscriber_callback(self._sub_callback, topic._name)
            self._state = STOP_STATE
    
