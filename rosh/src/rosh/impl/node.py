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
# Revision $Id: node.py 11682 2010-10-22 07:53:07Z kwc $

from __future__ import with_statement

import rosnode
import rospy

from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept, NSResourceList

class NodeInfo(object):
    
    def __init__(self, config, name, pubs, subs, srvs):
        self.name, self._pubs, self._subs, self._srvs = name, pubs, subs, srvs

        # we need to use the config from the relevant concept so that
        # we are allocating off of pre-existing instances. We also
        # need to fetch the relevant Namespace class for instantiating
        # resources.
        t = config.ctx.topics
        s = config.ctx.services
        topic_config = t._config
        service_config = s._config
        TopicNS = t._nstype
        ServiceNS = s._nstype

        self.pubs = NSResourceList('', topic_config, pubs, TopicNS)
        self.subs = NSResourceList('', topic_config, subs, TopicNS)
        self.srvs = NSResourceList('', service_config, srvs, ServiceNS)
        
    def _update(self, pubs, subs, srvs):
        """
        In-place update of resource lists
        """
        self._pubs, self._subs, self._srvs = pubs, subs, srvs
        self.pubs._set_resources(pubs)
        self.subs._set_resources(subs)
        self.srvs._set_resources(srvs)
        
    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        pubs, subs, srvs = self._pubs, self._subs, self._srvs
        buff = 'Node [%s]\n'%(self.name)
        if pubs:
            buff += "\nPublications: \n"
            buff += '\n'.join([" * %s"%(l) for l in pubs]) + '\n'
        else:
            buff += "\nPublications: None\n"
        if subs:
            buff += "\nSubscriptions: \n"
            buff += '\n'.join([" * %s"%(l) for l in subs]) + '\n'
        else:
            buff += "\nSubscriptions: None\n"        
        if srvs:
            buff += "\nServices: \n"
            buff += '\n'.join([" * %s"%l for l in srvs]) + '\n'
        else:
            buff += "\nServices: None\n"

        return buff

def node_info(config, node_name, node_info_obj=None):
    """
    @param config: Namespace config object to use for scoping.
    @type  config: NamespaceConfig
    @param node_name: ROS Name of Node
    @type  node_name: str
    @param node_info_obj: (optional) If a NodeInfo instance is
    provided, it will be updated with the resulting
    information. Otherwise, a new instance will be generated.
    @type  node_info_obj: NodeInfo
    """
    # go through the master system state first
    state = config.ctx.master.getSystemState()
    pubs = [t for t, l in state[0] if node_name in l]
    subs = [t for t, l in state[1] if node_name in l]
    srvs = [t for t, l in state[2] if node_name in l]
    if node_info_obj is None:
        return NodeInfo(config, node_name, pubs, subs, srvs)
    else:
        node_info_obj._update(pubs, subs, srvs)        
    
class NodeNS(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance. 
        @type  config: L{NamespaceConfig}
        """
        super(NodeNS, self).__init__(name, config)
        self._uri = None
        self._init_uri()
        self._info_obj = None

    def _list(self):
        """
        Override Namespace._list()
        """
        return rosnode.get_node_names(namespace=self._ns)

    def _kill(self):
        if self._process is not None:
            # kill attached process directly
            self._process._kill()
            self._process = None
        else:
            # TODO: enhance roslaunch API to allow more direct kill
            rosnode.kill_nodes([self._name])
    
    def _init_uri(self):
        if self._name and self._uri is None:
            try:
                self._uri = rosnode.get_api_uri(self._config.master.handle, self._name)
            except:
                pass
        
    def _show(self):
        """
        show() handler
        """
        show_graph(self._ns)
    
    def __repr__(self):
        return self.__str__()
        
    def __str__(self):
        """
        String representation of node. Provides node's URI.
        """
        if self._uri is None:
            self._init_uri()
        if self._uri is None:
            return self._ns
        else:
            return "Name: %s\nURI: %s"%(self._name, self._uri)

    def _info(self):
        """
        info() API. 
        
        @return: node info object that provides access to pubs, subs, and srvs
        @rtype: L{NodeInfo}
        """
        # if we don't cache the info obj, we would leak memory in the
        # shell each time the user calls info. This keeps the memory
        # allocation fixed.
        if self._info_obj is None:
            self._info_obj = node_info(self._config, self._name)
        else:
            node_info(self._config, self._name, self._info_obj)
        return self._info_obj

    def __call__(self):
        """
        Ping the node.
        """
        return rosnode.rosnode_ping(self._name, max_count=1, verbose=False)
    
class Nodes(Concept):

    def __init__(self, ctx, lock):
        super(Nodes, self).__init__(ctx, lock, NodeNS)

    def _show(self):
        show_graph('/')

def show_graph(ns):
    import subprocess
    cmd = ['rxgraph', '--nodens', ns]
    # TODO: check for error return code?
    subprocess.Popen(cmd, stderr=subprocess.PIPE)

