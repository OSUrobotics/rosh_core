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
# Revision $Id: plugin.py 11463 2010-10-08 19:00:27Z kwc $

import roslib
import roslib.packages
from rosh.impl.exceptions import ROSHException, InvalidPlugin, NoPlugin

# TODO: possibly store other data in here, like plugin help, etc...
class PluginContext(object):

    def __init__(self, rosh_globals, rosh_context, rosh_lock):
        # global symbol table for rosh module
        self.rosh_globals = rosh_globals
        # namespace ctx object of rosh
        self.ctx = rosh_context
        # common lock object for initialization
        self.rosh_lock = rosh_lock
        
        self._apis = {}

    def _register_api(self, plugin_api_id, callback):
        self._apis[plugin_api_id] = callback

    def _register_handler(self, plugin_api_id, args):
        callback = self._apis[plugin_api_id]
        callback(*args)
        
class PluginData(object):
    
    def __init__(self):
        self.apis = {}
        self.handlers = {}
    
    def add_handler(self, plugin_api_id, args):
        """
        @param plugin_api_id: identifier for API to register with
        @type  plugin_api_id: str
        @param args: arguments for plugin API
        @type  args: [any]
        """
        if plugin_api_id not in self.handlers:
            self.handlers[plugin_api_id] = []
        self.handlers[plugin_api_id].append(args)

    def add_api(self, plugin_api_id, callback):
        """
        Register plugin API that downstream plugins can add handlers for.

        @param plugin_api_id: identifier for this API
        @type  plugin_api_id: str
        @param callback: callback function to invoke when handlers register with this API
        @type  callback: fn
        """
        self.apis[plugin_api_id] = callback
               
def reentrant_load(loaded_symbols, globals_):
    """
    Utility routine to allow plugins that have already loaded to
    initialize a new globals_ dictionary.
    """
    if loaded_symbols is not None:
        if globals_ is not None:
            globals_.update(loaded_symbols)
        return True
    else:
        return False

def globals_load(plugin_context, globals_, symbols):
    """
    Utility routine for plugins to load their symbols into the
    appropriate global symbol tables
    """
    for g in [plugin_context.rosh_globals, globals_]:
        if g is not None:
            g.update(symbols)
    
def load_plugin(name, plugin_context, globals_=None):
    """
    Load ROSH plugin from another ROS package.

    load_plugin() loads the plugin into the rosh global symbol
    table. If globals_ is specified, plugin symbols will also be
    loaded into the provided dictionary.

    @param globals_: global symbol table to additionally load plugin to.
    @type  globals_: dict
    @raise NoPlugin: if plugin does not exist
    @raise InvalidPlugin: if plugin fails to load properly
    """
    
    # remap friendly names to actual implementation. this is a bit of
    # abstraction leakage and should instead be dynamically registered
    # (though much more expensive).

    #TODO: this is a hack until rosh_testing is a proper framework/plugin"
    if name == 'rosh.impl.testing':
        import rosh.impl.testing
        rosh.impl.testing.load_rosh_plugin('rosh.impl.ros_testing', plugin_context, globals_)
        return
    
    try:
        # make sure plugin exists for accurate error reporting
        roslib.packages.get_pkg_dir(name)
    except roslib.packages.InvalidROSPkgException, e:
        raise NoPlugin("invalid plugin [%s]: no ROS package named [%s]"%(name, name))
    try:
        roslib.load_manifest(name)
    except roslib.packages.InvalidROSPkgException, e:
        raise InvalidPlugin("Failed to locate dependencies of plugin [%s]: \n[%s]"%(name, str(e)))
    try:
        m = __import__(name)
    except ImportError:
        raise InvalidPlugin("invalid plugin [%s]: python module [%s] import failed"%(name, name))
    try:
        loader = getattr(m, 'rosh_plugin_load')
    except AttributeError, e:
        raise InvalidPlugin("invalid plugin [%s]: plugin is missing rosh_plugin_load() entry point"%(name))

    try:
        plugin_data = loader(plugin_context, globals_)
    except Exception, e:
        raise InvalidPlugin("invalid plugin [%s]: plugin raised Exception on load: %s"%(name, e))
    errors = []
    if plugin_data is not None:
        
        # Wire up plugins. Although the plugins could do this
        # themselves, de-coupling provides better protection against
        # future implementation changes and is also easier to write
        # tests for.

        # - We load as much as we can, record the errors, then
        #   re-raise at the end.
        for plugin_api_id, callback in plugin_data.apis.iteritems():
            try:
                plugin_context._register_api(plugin_api_id, callback)
            except Exception, e:
                errors.append(e)

        for plugin_api_id, args in plugin_data.handlers.iteritems():
            try:
                plugin_context._register_handler(plugin_api_id, args)
            except Exception, e:
                errors.append(e)

    if errors:
        error_str = [str(e) for e in errors]
        raise ROSHException("errors loading plugin [%s]: \n%s"%(name, '\n'.join(error_str)))
