# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
# Revision $Id: roslaunch_logs.py 7331 2009-12-15 22:45:49Z kwc $

"""
Implementation for roslaunch-logs command-line utility.
"""

import os
import sys
import time
import traceback

import roslib.rosenv
import roslib.scriptutil

NAME = 'roslaunch-logs'

def get_run_id():
    try:
        param_server = roslib.scriptutil.get_param_server()
        code, msg, val = param_server.getParam('/roslaunch', '/run_id')
        if code == 1:
            return val
    except: # cannot contact parameter server
        pass

def logs_main():
    from optparse import OptionParser
    parser = OptionParser(usage="usage: %prog", prog=NAME)
    options, args = parser.parse_args()
    if args:
        parser.error("%s takes no arguments"%NAME)
        
    log_dir = roslib.rosenv.get_log_dir()
    if not log_dir:
        print >> sys.stderr, "Cannot determine ROS log directory"
        sys.exit(1)
        
    run_id = get_run_id()
    if not run_id:
        # go ahead and print the log directory
        print >> sys.stderr, "No active roscore"
        print log_dir
        sys.exit(2)

    print os.path.join(log_dir, run_id)
        

    
if __name__ == '__main__':
    logs_main()
