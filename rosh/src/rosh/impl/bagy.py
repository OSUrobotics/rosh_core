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
# Revision $Id: bagy.py 11681 2010-10-22 07:08:51Z kwc $

from __future__ import with_statement

"""
YAML-based bag formats.
"""

#TODO: convert to ABCs
#TODO: orderedkey bagy ?
#TODO: multimap bagy?

import collections
import yaml

import roslib.message

import rosh
from rosh.impl.exceptions import ROSHException
from rosh.impl.topic import TopicNS

def create(msg_repr, type_, filename=None):
    """
    Create single message from YAML/dictionary representation.

    @param type_: Message class
    @type  type_: class (Message subclass)
    @param msg_repr: Message in string or dictionary representation
    @type  msg_repr: str or dict
    @param filename: Name of file associated with msg_repr for debugging.
    @type  filename: str
    """
    if type(msg_repr) == str:
        msg_repr = yaml.load(msg_repr)
    if type(msg_repr) != dict:
        if filename:
            raise ValueError("msg_repr file [%s] does not contain a YAML dictionary"%filename)
        else:
            raise ValueError("msg_repr must contain a dictionary")
    m = type_()
    # TODO: add new API to roslib.message that doesn't assume a rostopic-like API source
    roslib.message.fill_message_args(m, [msg_repr])
    return m

ZERO_STATE = 0
RECORD_STATE = 1
STOP_STATE = 2

class _Bagy(object):

    def __init__(self, name, stream, type_):
        """
        ctor.
        """
        self.name    = name
        self.type_   = type_
        self._stream = stream
        self._buff   = None
        self._yaml_iter = None
        self.closed = False

        # for recording
        self._state = ZERO_STATE
        self._record = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __iter__(self):
        for doc in yaml.load_all(self._stream):
            if doc is not None:
                yield create(doc, self.type_)

    def write(self, msg):
        """
        Write message to bagy
        @param msg: messsage to add
        @type  msg: self.type_
        """
        if not isinstance(msg, roslib.message.Message):
            raise ValueError("msg is not a message instance")
        self._stream.write(roslib.message.strify_message(msg) + '\n---\n')

    def read(self):
        """
        Read *all* messages from bagy. 
        
        @return: messages
        @rtype: [self.type_]
        """
        return [create(d, self.type_) for d in yaml.load_all(self._stream) if d is not None]

    def next(self):
        """
        Get the next message in the bagy
        
        @return: Message instance
        @rtype: self.type_
        """
        if self._yaml_iter is None:
            self._yaml_iter = yaml.load_all(self._stream)
        v = self._yaml_iter.next()
        if v is None:
            return None
        return create(v, self.type_)

    def close(self):
        """
        Close resources associated with Bagy
        """
        self.closed = True
        if self._state == RECORD_STATE:
            self.stop()
            self._state = ZERO_STATE
            self._record = None

        self._stream.close()
        if self._yaml_iter is not None:
            self._yaml_iter.close()

    def record(self, topic):
        if self.closed:
            raise ROSHException("closed")
        if self._record is not None:
            raise ROSHException("already recording topic %s"%(self._record._name))
            
        if isinstance(topic, TopicNS):
            pass
        elif type(topic) == str:
            ctx = rosh.get_default_plugin_context().ctx
            topic = ctx.topics[topic]

        # TODO: clean this up once I add proper subscribe() to topic
        topic._add_subscriber_callback(self.write)
        self._record = topic
        self._state = RECORD_STATE

    def start(self):
        if self.closed:
            raise ROSHException("closed")
        if self._record is None:
            raise ROSHException("not recording")
        if self._state == STOP_STATE:
            self._record._add_subscriber_callback(self.write)
            self._state = RECORD_STATE
            
    def stop(self):
        if self._record is None:
            raise ROSHException("not recording")
        
        if self._state == RECORD_STATE:
            self._record._remove_subscriber_callback(self.write)
            self._state = STOP_STATE
    
            
class _MapBagy(collections.Mapping):
    """
    Bagy where messages are indexed by labels (keys).
    """
    def __init__(self, name, stream, type_):
        """
        ctor.
        """
        self.name    = name
        self.type_   = type_
        self._stream = stream
        self._buff   = None
        self._db     = None
            
    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __iter__(self):
        if self._db is None:
            self._load_db()
        for key, d in self._db.iteritems():
            yield key, create(d, self.type_)

    # collections.Mapping
    def __contains__(self, key):
        if self._db is None:
            self._load_db()
        return key in self._db

    # collections.Mapping
    def __eq__(self, other):
        # easy but expensive way
        return self.items().__eq__(other.items())
    
    # collections.Mapping
    def __ne__(self, other):
        # easy but expensive way
        return self.items().__ne__(other.items())

    # collections.Mapping
    def keys(self):
        if self._db is None:
            self._load_db()
        return self._db.keys()
    
    # collections.Mapping
    def items(self):
        return list(self.iteritems())

    # collections.Mapping    
    def values(self):
        return list(self.itervalues())

    def iterkeys(self):
        if self._db is None:
            self._load_db()
        return self._db.iterkeys()

    def iteritems(self):
        return self.__iter__()

    def itervalues(self):
        if self._db is None:
            self._load_db()
        for d in self._db.itervalues():
            yield create(d, self.type_)

    def write(self, *args, **kwds):
        """
        write message into bagy. Must either pass in label and msg instance::

            b.write('key', m)

        or, pass in Python keywords, where keys are the labels::
        
            b.write(key1=m1, key2=m2)            
        """
        if args:
            key, msg = args
            if not isinstance(key, basestring):
                raise TypeError("label must be a string")
            if not isinstance(msg, roslib.message.Message):
                raise ValueError("msg is not a message instance")
            self._stream.write('%s:%s\n'%(key,
                                            roslib.message.strify_message(msg, indent='  ')))
        elif kwds:
            for key, msg in kwds.iteritems():
                if not isinstance(key, basestring):
                    raise TypeError("label must be a string")
                if not isinstance(msg, roslib.message.Message):
                    raise ValueError("msg is not a message instance")
                self._stream.write('%s:%s\n'%(key,
                                                roslib.message.strify_message(msg, indent='  ')))
        else:
            raise TypeError("must provide an arguments or keywords to write()")

    def _load_db(self):
        """
        Load YAML dictionary into bagy instance. We don't convert the
        dicts to messages until they are requested.
        """
        db = {}
        for d in yaml.load_all(self._stream):
            db.update(d)
        self._stream.close()
        self._db = db
        
    def __setitem__(self, key, value):
        """
        Add item to label bagy as a map, e.g.::
        
            b = keybagy('foo.bagy', 'w')
            b[key] = msg
        """
        self.write(key, value)

    # collections.Mapping
    def get(self, key, default=None):
        """
        Get value from bagy, with optional default, e.g.::
        
            b = keybagy('foo.bagy', 'r', msg.std_msgs.String)
            msg = b.(key, altmsg)
        """
        if self._db is None:
            self._load_db()
        if key in self._db:
            return create(self._db[key], self.type_)
        else:
            return default
        
    # collections.Mapping
    def __len__(self, key):
        if self._db is None:
            self._load_db()
        return self._db.__len__()

    # collections.Mapping
    def __getitem__(self, key):
        """
        Access label bagy as a map, e.g.::
        
            b = keybagy('foo.bagy', 'r', msg.std_msgs.String)
            msg = b[key]
        """
        if self._db is None:
            self._load_db()
        return create(self._db[key], self.type_)

    def close(self):
        self._stream.close()
        if self._db is not None:
            del self._db

    def read(self):
        """
        Read *all* messages from label bagy. This exhausts the bagy
        and future methods that access data from the bagy will fail.
        
        @return: messages
        @rtype: dict{str: self.type_}
        """
        if self._db is None:
            self._load_db()

        retval = {}
        for k, v in self._db.iteritems():
            retval[k] = create(v, self.type_)
        del self._db
        self._db = None
        return retval



class Bagy(_Bagy):
    def __init__(self, filename, mode='r', type_=None):
        """
        @param filename: path to file
        @type  filename: str
        @param mode: open mode. Valid modes are 'r' (read), 'w' (write), and 'a' (append). 
        @type  mode: str
        @param type_: Message type stored in bagy
        @type  type_: type(roslib.message.Message)
        """
        if mode in ['r', 'w', 'a']:
            if mode == 'r' and type_ is None:
                raise ValueError("type_ must be specified when reading from bagy files")
            self.mode = mode
            super(Bagy, self).__init__(filename, open(filename, mode), type_)
        else:
            raise ValueError("mode is invalid")
    
class MapBagy(_MapBagy):
    def __init__(self, filename, mode, type_=None):
        """
        @param filename: path to file
        @type  filename: str
        @param mode: open mode. Valid modes are 'r' (read), 'w' (write), and 'a' (append). 
        @type  mode: str
        @param type_: Message type stored in key bagy
        @type  type_: type(roslib.message.Message)
        """
        if mode in ['r', 'w', 'a']:
            if mode == 'r' and type_ is None:
                raise ValueError("type_ must be specified when reading from bagy files")
            self.mode = mode
            super(MapBagy, self).__init__(filename, open(filename, mode), type_)
        else:
            raise ValueError("mode is invalid")

