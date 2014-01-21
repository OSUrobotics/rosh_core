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
# Revision $Id: packages.py 11672 2010-10-22 00:18:14Z kwc $

from __future__ import with_statement

import os
import stat
import sys

import roslib.manifest
import roslib.packages
import roslib.stack_manifest
import roslib.stacks

# TODO: get rid of Namespace and Concept entirely as that really doesn't apply here and just requires workarounds
from rosh.impl.namespace import Namespace, Concept, ResourceList
# TODO: enable packages.std_msgs.msgs.String
from rosh.impl.msg import Msgs

class ManifestResource(object):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance with additional 'listener' attribute. 
        @type  config: L{NamespaceConfig}
        """
        self.name = name
        # this is an artifact of "Concepts" having a 'root' with an
        # empty key, which is non-sensical in this implementation.
        # this could be resolved by not reusing the Namespace/Concept
        # code directly
        if not name:
            return
        self._config = config

        self.path = self._get_path()
        self.manifest = self._get_manifest(self.path)

    def _get_path(self):
        raise NotImplemented

    def _get_manifest(self, path):
        raise NotImplemented

    def _get_depends1(self):
        # we do a str(x) to flatten into str form instead of
        # StackDepend vs. Depend, which have different attributes
        return ResourceList(self._config, [str(x) for x in self.manifest.depends], self.__class__)

    depends1 = property(_get_depends1)
    
    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        return self.name

class AttrDict(object):
    def __init__(self, d):
        self.__dict__.update(d)
        
class Package(ManifestResource):
    def __init__(self, name, config):
        super(Package, self).__init__(name, config)

    def _get_path(self):
        return roslib.packages.get_pkg_dir(self.name)

    def _get_manifest(self, path):
        return roslib.manifest.parse_file(os.path.join(path, roslib.manifest.MANIFEST_FILE))

    def _get_msgs(self):
        try:
            # test if there are, in fact, msg files
            if self.name in self._config.ctx.msg:
                # if so, retrieve from msg
                return self._config.ctx.msg[self.name]
        except AttributeError:
            # msg property is not guaranteed to exist
            pass
        return None

    def _get_srvs(self):
        try:
            # test if there are, in fact, srv files
            if self.name in self._config.ctx.srv:
                # if so, retrieve from msg
                return self._config.ctx.srv[self.name]
        except AttributeError:
            # srv property doesn't exist if we are not online
            pass
        return None

    #TODO: can we make nodes and launches plugins on top of packages,
    #that way rosh can be provided in the separated ROS stack.
    def _get_nodes(self):
        paths = list_resources_by_dir(self.path, node_filter)
        node_types = [os.path.basename(p) for p in paths]
        d = dict([(launchablekey(t), LaunchableNode(self._config.ctx, self.name, t)) for t in node_types])
        return AttrDict(d)

    def _get_launches(self):
        paths = list_resources_by_dir(self.path, launch_filter)
        launch_types = [os.path.basename(p) for p in paths]
        d = dict([(launchablekey(t), LaunchableLaunch(self._config.ctx, self.name, t)) for t in launch_types])
        return AttrDict(d)

    # use singular to match 'msg' global
    msg = property(_get_msgs)
    srv = property(_get_srvs)
    nodes = property(_get_nodes)
    launches = property(_get_launches)

def launchablekey(k):
    return k.replace('.', '_')

try:
    import roslaunch
except ImportError:
    print >> sys.stderr, "cannot import roslaunch, launching is disabled"
    class FakeLaunch(object):
        def Node(self):
            raise Exception("cannot import roslaunch, launching is disabled")
    roslaunch = FakeLaunch()
    
class LaunchableNode(object):
    def __init__(self, ctx, package, type_):
        self.ctx = ctx
        self.package = package
        self.type = type_
    def __call__(self, *args, **kwds):
        # launch returns list of Nodes that were launched, so unwrap
        return self.ctx.launch(self.as_Node(), args=args, remap=kwds)[0]
        
    def as_Node(self):
        return roslaunch.Node(self.package, self.type)
    def __str__(self):
        return launchablekey(self.type)
    def _launch(self):
        return self.ctx.launch(self.as_Node())

        
class LaunchableFile(object):
    def __init__(self, ctx, launch_file):
        self.ctx = ctx
        self.launch_file = launch_file
    def __call__(self):
        raise NotImplemented
    def __str__(self):
        return launchablekey(self.launch_file)
    def _launch(self):
        raise NotImplemented

def node_filter(path, filename):
    """
    Node file filter function for list_resources. Filter functions
    return the resource path if the filter matches, None otherwise.

    @return: resource path (if match) or None
    """
    path = os.path.join(path, filename)
    s = os.stat(path)
    if (s.st_mode & stat.S_IRWXU == stat.S_IRWXU):
        return path

def launch_filter(path, filename):
    """
    roslaunch file filter function for list_resources. Filter
    functions return the resource path if the filter matches, None
    otherwise.

    @return: resource path (if match) or None
    """
    if filename.endswith('.launch') or filename.endswith('.test'):
        return os.path.join(path, filename)
        
# TODO: move into roslib for ROS 1.4. Right now rosh is targetting ROS 1.2 compatibility
def list_resources(pkg, filter_fn):
    return list_resources_by_dir(roslib.packages.get_pkg_dir(pkg), filter_fn)
def list_resources_by_dir(pkg_dir, filter_fn):
    resources = []
    for p, dirs, files in os.walk(pkg_dir):
        for f in files:
            resource_path = filter_fn(p, f)
            if resource_path is not None:
                resources.append(resource_path)
        if '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')
    return resources
        
class Stack(ManifestResource):
    def __init__(self, name, config):
        super(Stack, self).__init__(name, config)

    def _get_path(self):
        return roslib.stacks.get_stack_dir(self.name)

    def _get_manifest(self, path):
        return roslib.stack_manifest.parse_file(os.path.join(path, roslib.stack_manifest.STACK_FILE))

    def _get_packages(self):
        # we have to use the config from packages in order to get access to its cache
        return ResourceList(self._config.ctx.packages._config, roslib.stacks.packages_of(self.name), Package)

    packages = property(_get_packages)

class ManifestResources(Concept):
    def __init__(self, ctx, lock, impl_class, list_fn):
        super(ManifestResources, self).__init__(ctx, lock, impl_class)

        # lazy init for performance reasons
        self._list_cache = None
        self._list_fn = list_fn
        self._impl_class = impl_class

    def _getAttributeNames(self):
        if self._list_cache is None:
            self._list_cache = self._list_fn()
        return self._list_cache

    def __getattribute__(self, key):
        if key.startswith('_') or key == 'trait_names':
            return object.__getattribute__(self, key)
        else:
            return self.__getitem__(key)

    def __iter__(self):
        return (getattr(self, k) for k in self._getAttributeNames())

    def _get_entry(self, key):
        # generate cache on demand
        if self._list_cache is None:
            self._list_cache = self._list_fn()

        # in the future we could try and do a redectect, but not high
        # priority nor common
        if not key in self._list_cache:
            raise AttributeError(key)
        cache = self._config.cache
        with self._config.lock:
            if key in cache:
                obj = cache[key]
            else:
                # create a new instance of ourselves. This requires
                # subclasses to have same constructor args.
                obj = self._impl_class(key, self._config)
                cache[key] = obj
        return obj

    def __getitem__(self, key):
        """
        Dictionary-style accessor 
        """
        if key in self._config.cache:
            return self._config.cache[key]
        else:
            val = self._get_entry(key)
            return val

class Packages(ManifestResources):

    def __init__(self, ctx, lock,):
        super(Packages, self).__init__(ctx, lock, Package, roslib.packages.list_pkgs_by_path)

class Stacks(ManifestResources):

    def __init__(self, ctx, lock):
        super(Stacks, self).__init__(ctx, lock, Stack, roslib.stacks.list_stacks)
