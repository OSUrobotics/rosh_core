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
# Revision $Id: namespace.py 11682 2010-10-22 07:53:07Z kwc $
"""
Generic namespace model for ROS. This powers most of rosh by enabling
iPython tab-complete on a ROS namespace'd resource (e.g. topics,
services, parameters).
"""

#TODO: remapping support

from __future__ import with_statement

import roslib.names

class Context(object):
    """
    ROSH context instance. Instead of globals, we need a context
    instance that implementation can store state in.

    Plugins may store properties on this object as long as they
    include their name as a prefix.
    """
    
    def __init__(self):
        # only guaranteed property
        self.master = None
        self.plugin_handlers = {}
        
    def add_plugin_handler(self, key, args):
        self.plugin_handlers[key] = args

class NamespaceConfig(object):
    """
    NamespaceConfig is a configuration object for a L{Namespace} instance,
    storing common data structures and objects that all L{Namespace}
    instances need.
    """
    
    def __init__(self, ctx, lock):
        """
        @param ctx: rosh context object
        @type  ctx: dict
        @param lock: lock for controlling access to cache
        @param master: ROS master handle, if applicable
        @type  master: rosgraph.MasterApi
        """
        self.cache = {}
        self.lock = lock
        self.ctx = ctx
        self.master = ctx.master

class Namespace(object):
    """
    Namespace provides lightweight accessors to a ROS namespace and its data. The behavior of the namespace
    object is determined by a L{NamespaceConfig} instance.
    """
    
    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        self._name = name
        self._ns = name + '/'
        self._config = config
        
    def _list(self):
        """
        Subclasses should override.
        """
        raise NotImplemented

    def _props(self):
        """
        Subclasses should override if they have props
        """
        return []
    
    def _init(self, *args):
        """
        Subclasses should override.
        """
        raise NotImplemented
    
    def _getAttributeNames(self):
        return list(set([s[len(self._ns):].split('/')[0] for s in self._list()]))
    
    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self.__getitem__(key)

    def __iter__(self):
        return (getattr(self, k) for k in self._getAttributeNames())

    def _get_entry(self, key):
        """
        By default, creates a new Namespace instance of the correct
        subclass. Subclasses may wish to override.
        """
        cache = self._config.cache
        with self._config.lock:
            if key in cache:
                obj = cache[key]
            else:
                # create a new instance of ourselves. This requires
                # subclasses to have same constructor args.
                obj = self.__class__(key, self._config)
                cache[key] = obj
        return obj

    def __getitem__(self, key):
        """
        Dictionary-style accessor 
        """
        key = roslib.names.ns_join(self._ns, key)
        if key in self._config.cache:
            return self._config.cache[key]
        else:
            val = self._get_entry(key)
            return val
    
class Concept(object):

    def __init__(self, ctx, lock, nstype):
        """
        @param ctx: Context instance
        @type  ctx: L{Context}
        @param lock: Resource lock for modifying ctx resources
        @type  lock: threading.Lock
        @param nstype: Namespace class
        @type  nstype: class
        """
        self._master = ctx.master

        self._config = NamespaceConfig(ctx, lock)
        self._root = nstype('', self._config)
        self._nstype = nstype
        
    def _init(self, *args):
        """
        Subclasses should override (if necessary)
        """
        raise NotImplemented

    def _getAttributeNames(self):
        return self._root._getAttributeNames()
        
    def _props(self):
        """
        Subclasses should override if they have props
        """
        return []
    
    def __iter__(self):
        return self._root.__iter__()
        
    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self._root.__getattribute__(key)

    def __getitem__(self, key):
        """
        Dictionary-style accessor for topics
        """
        return self._root.__getitem__(key)

class ResourceList(object):
    """
    A ResourceList presents a IPython tab-completable list of
    resources as if they are attributes.  The general form of
    ResourceList only presents a flat list of resources. For a
    ResourceList that is namespace-aware, use L{NSResourceList}.
    
    ResourceList can share a config with a Namespace instance, which
    makes them useful as views against larger pools of resources.
    """

    def __init__(self, config, resources, resource_class):
        """
        @param config: NamespaceConfig instance
        @param resource_class: Class to instantiate to create new
        resources if resource has not already been created.
        @type  resource_class: Class
        """
        
        self._resources = resources
        self._config = config
        self._resource_class = resource_class

    def __repr__(self):
        return '\n'.join(self._resources)
    
    def __iter__(self):
        """
        Not thread-safe. If ResourceList is modified during iteration, this will fail
        """
        for r in self._resources:
            yield self._get_entry(r)

    def _list(self):
        return self._resources
    
    def _getAttributeNames(self):
        return self._resources

    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self.__getitem__(key)

    def __getitem__(self, key):
        if key in self._config.cache:
            return self._config.cache[key]
        else:
            val = self._get_entry(key)
            return val
        
    def _get_entry(self, key):
        # in the future we could try and do a redetect, but not high
        # priority nor common
        if not key in self._resources:
            raise AttributeError(key)
        cache = self._config.cache
        with self._config.lock:
            if key in cache:
                obj = cache[key]
            else:
                # create a new instance of resource. resource_class
                # must obey general resource constructor API.
                obj = self._resource_class(key, self._config)
                cache[key] = obj
        return obj

class NSResourceList(object):
    """
    NSResourceList is a constrained version of a Namespace instance and
    can be used to create a view against a larger Namespace. It can
    share a config with a Namespace instance.
    """

    def __init__(self, name, config, resources, resource_class):
        self._ns = name + '/'

        self._resources = resources
        self._config = config
        self._resource_class = resource_class

    def _set_resources(self, resources):
        """
        Change resource list
        """
        self._resources = resources
        
    def __repr__(self):
        return '\n'.join([x for x in self._resources if x.startswith(self._ns)])
    
    def _list(self):
        return self._resources
    
    def __iter__(self):
        """
        Not thread-safe. If ResourceList is modified during iteration, this will fail
        """
        for r in self._resources:
            yield self._get_entry(r)

    def _getAttributeNames(self):
        return list(set([s[len(self._ns):].split('/')[0] for s in self._list()]))

    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self.__getitem__(key)

    def __getitem__(self, key):
        key = roslib.names.ns_join(self._ns, key)
        # we additionally constrain key to be in self._resources
        # because cache is shared with the full Namespace. We use
        # NSResourceList references for any partial (parent namespace)
        # references.
        if key in self._resources and key in self._config.cache:
            return self._config.cache[key]
        else:
            val = self._get_entry(key)
            return val
        
    def _get_entry(self, key):
        # in the future we could try and do a redetect, but not high
        # priority nor common
        res = self._resources
        cache = self._config.cache
        with self._config.lock:
            if key in res:
                if key in cache:
                    obj = cache[key]
                else:
                    # create new instance of resource. resource must object construction API
                    obj = self._resource_class(key, self._config)
                    cache[key] = obj
            else:
                # descend namespace, do *not* cache in the main cache
                # as that would pollute the Namespace cache. This may
                # be a performance issue that we should provide a
                # separate cache for.
                if any(r for r in res if r.startswith(key+'/')):
                    obj = self.__class__(key, self._config, res, self._resource_class)
                else:
                    # do not spawn for keys not within our namespace
                    return None
        return obj
    

def rostype(ns_obj, type_=None):
    """
    Get or set the ROS type of an object. This is generally associated
    with Namespace instances that have associated types.
    
    @param type_: (optional) set type of object to type_, if possible
    @raise TypeError: if type_ is not of the correct type
    @raise ROSHException: if type cannot be set
    """
    if type_ is None:
        if hasattr(ns_obj, '_get_type'):
            return ns_obj._get_type()
        else:
            raise ROSHException("object has no ROS type")
    else:
        if hasattr(ns_obj, '_set_type'):
            ns_obj._set_type(type_)
        else:
            raise ROSHException("object has no ROS type")


def info(ns_obj):
    if hasattr(ns_obj, '_info'):
        return ns_obj._info()
    else:
        raise ROSHException("object not not support _info")

