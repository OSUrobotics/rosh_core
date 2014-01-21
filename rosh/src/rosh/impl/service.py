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
# Revision $Id: service.py 13210 2011-02-14 01:05:08Z kwc $

from __future__ import with_statement

import sys

import roslib.message

import rosgraph.masterapi
import rosmsg
import rosservice
import rospy

from rosh.impl.exceptions import ROSHException
from rosh.impl.namespace import Namespace, Concept

def Service(service_name, service_type, handler):
    if type(service_name) == Namespace:
        return rospy.Service(service_name._name, service_type, handler)
    else:
        return rospy.Service(service_name, service_type, handler)

def get_service_class_by_service_name(service_name):
    #TODO: this is technically a bug because it doesn't use the ctx's
    #master handle
    type_name = rosservice.get_service_type(service_name)
    if type_name:
        val = roslib.message.get_service_class(type_name, reload_on_error=True)
        if not val:
            pkg, base_type = roslib.names.package_resource_name(type_name)
            print >> sys.stderr, """Cannot retrieve type [%s].
Please type 'rosmake %s'
"""%(type_name, pkg)
        else:
            return val

class ServiceInfo(object):
    def __init__(self, name, node, uri):
        self.name, self.node, self.uri = name, node, uri
    def __repr__(self):
        return self.__str__()
    def __str__(self):
        return "Name: %s\nNode: %s\nURI: %s"%(self.name, self.node, self.uri)
        

def service_info(config, name):
    master = config.ctx.master
    try:
        uri = master.lookupService(name)
    except rosgraph.masterapi.Error:
        return None

    #TODO: convert rosservice to take in master handle
    #can't use rosservice because it doesn't let us pass in master
    #handle
    _, _, srvs = master.getSystemState()
    l = [nodelist for s, nodelist in srvs if s == name][0]
    node_name = l[0]
    node = config.ctx.nodes[node_name]
    return ServiceInfo(name, node, uri)

class ServiceNS(Namespace):

    def __init__(self, name, config):
        """
        ctor.
        @param config: Namespace configuration instance. Safe to set
        to None if self._config is initialize proper to calling any
        methods.
        @type  config: L{NamespaceConfig}
        """
        super(ServiceNS, self).__init__(name, config)
        self._type = self._service_info = None
        self._init_type()
        self._is_wait_for_service = True # per API review

    def _info(self):
        """
        info() API
        """
        if self._service_info is None:
            self._service_info = service_info(self._config, self._name)
        return self._service_info
    
    def _list(self):
        """
        Override Namespace._list()
        """
        # hide rosh services
        return [s for s in rosservice.get_service_list(namespace=self._ns) if not s.startswith('/rosh_')]

    def _get_type(self):
        "rostype() API"
        return self._type
        
    def _set_type(self, type_):
        "rostype() API"
        self._init_type(type_)
    
    def _init_type(self, t=None):
        if self._type is not None and t is None:
            # already initialized
            return
        
        if t is None:
            try:
                self._type = get_service_class_by_service_name(self._name)
                self._type_name = self._type._type
            except:
                # right now the contract of init_type is to
                # soft-fail and require callers to check _type
                # state
                self._type = self._type_name = None
        else:
            if type(t) == str:
                t = roslib.message.get_service_class(t)
                if t is None:
                    raise ValueError("cannot load service type: %s"%(t))
            else:
                if not type(t) == type:
                    raise ValueError("invalid service type: %s"%(type(t)))
            self._type = t
            self._type_name = t._type
        
    def __repr__(self):
        return self.__str__()

    def __str__(self):
        if self._type is None:
            self._init_type()
        if self._type is None:
            return self._ns
        else:
            return rosmsg.get_srv_text(self._type._type)

    def _props(self):
        return ['wait_for_service']

    def _set_wait_for_service(self, val):
        self._is_wait_for_service = val
        
    def __call__(self, *args, **kwds):
        """
        Call service
        """
        # lazy-init
        if self._type is None:
            self._init_type()
        if self._type is None:
            raise ROSHException("cannot call service [%s]: unknown type"%(self._ns))
        # TODO: cache ServiceProxy objects for faster creation
        # TODO: optimize wait_for_service to use ctx.master for faster calling
        if self._is_wait_for_service:
            rospy.wait_for_service(self._ns)
        request = self._type._request_class(*args, **kwds)
        return rospy.ServiceProxy(self._ns, self._type)(request)
    
class Services(Concept):

    def __init__(self, ctx, lock):
        super(Services, self).__init__(ctx, lock, ServiceNS)
