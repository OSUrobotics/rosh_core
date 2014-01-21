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
# Revision $Id: testing.py 11293 2010-09-28 01:17:31Z kwc $

import unittest

import rosh.plugin

class Test(unittest.TestCase):
    
    def __init__(self):
        pass

    def runTest(self):
        # define to keep TestCase happy
        pass

    def add_coverage(self, module):
        #TODO: take code from rostest for spinning up coverage
        pass

    def __call__(self, test, msg=None):
        if not test:
            if msg:
                self.fail(msg)
            else:
                self.fail("test failed: %s"%(test))
    
    
# we use the general plugin loading framework to initialize to keep things tidy
_loaded_symbols = None
def load_rosh_plugin(name, plugin_context, globals_=None):
    global _loaded_symbols
    if rosh.plugin.reentrant_load(_loaded_symbols, globals_):
        return

    _loaded_symbols = {
        'test': Test(),
        }
    # save these symbols in the context
    rosh.plugin.globals_load(plugin_context, globals_, _loaded_symbols)


