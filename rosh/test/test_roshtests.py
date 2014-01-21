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
# Revision $Id: test_roshtests.py 11680 2010-10-22 07:04:07Z kwc $

PKG = 'rosh'
import roslib; roslib.load_manifest(PKG)

import os
import sys
import struct
import traceback
import unittest

def list_tests(d):
    """
    List all '.roshtest' files in a directory
    
    @return: list of (test_name, test_path) tuples
    """
    tests = []
    for p in os.listdir(d):
        if p.endswith('.roshtest'):
            tests.append( (p[:-len('.roshtest')], os.path.join(d, p)) )
    return tests

def tc_setUp(self):
    """
    Common setUp of test cases
    """
    pass

def tc_tearDown(self):
    """
    Common tearDown of test cases
    """
    pass

def tc_roshtestRunner(test_name, test_file):
    def fn(self):
        # load roshlet
        print "running", test_name
        plugins = ['rosh.impl.testing']
        import rosh.impl.roshlets
        try:
            rosh.impl.roshlets.standalone(test_name, test_file, plugins)
            print "done"
        except AssertionError, e:
            # unwrap the traceback info so that we can focus on the
            # actual assert that failed
            exc_type, exc_value, exc_traceback = sys.exc_info()
            lines = traceback.extract_tb(exc_traceback)
            err_msgs = []
            for f, lineno, meth, msg in lines:
                # TODO: this test can overmatch
                if test_name in f:
                    err_msgs.append("%s:%s: %s"%(test_name, lineno, msg))
            self.fail('\n'.join(err_msgs))
    return fn
    
def create_TestCase(test_name, test_file):
    test_fn = 'test_%s'%test_name
    classdict = { 'setUp': tc_setUp, 'tearDown': tc_tearDown,
                  test_fn: tc_roshtestRunner(test_name, test_file),}
    return type('%sRoshTest'%test_name,(unittest.TestCase,),classdict)    

if __name__ == '__main__':
    import rostest

    import roslib.packages
    d = os.path.join(roslib.packages.get_pkg_dir(PKG), 'test')
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    if len(sys.argv) > 1:
        tests = sys.argv[1:]
        test_names = [os.path.basename(t)[:-len('.roshtest')] for t in tests]
        tests = zip(test_names, tests)
    else:
        tests = list_tests(d)
    for test_name, test_path in tests:
        #rostest.unitrun(PKG, sys.argv[0], create_TestCase(test_name, test_path))
        tc = create_TestCase(test_name, test_path)
        tests = loader.loadTestsFromTestCase(tc)
        suite.addTests(tests)

    unittest.TextTestRunner().run(suite)
