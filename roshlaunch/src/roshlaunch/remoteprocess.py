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
# Revision $Id: remoteprocess.py 10235 2010-07-02 00:01:48Z kwc $

"""
Process handler for launching ssh-based roslaunch child processes.
"""

from __future__ import with_statement

import os
import sys
import socket
import traceback
import xmlrpclib

from roslaunch.core import printlog, printerrlog
import roslaunch.pmon
import roslaunch.server

import logging
_logger = logging.getLogger("roslaunch.remoteprocess")

# #1975 timeout for creating ssh connections
TIMEOUT_SSH_CONNECT = 30.

def ssh_check_known_hosts(ssh, address, port, username=None, logger=None):
    """
    Validation routine for loading the host keys and making sure that
    they are configured properly for the desired SSH. The behavior of
    this routine can be modified by the ROSLAUNCH_SSH_UNKNOWN
    environment variable, which enables the paramiko.AutoAddPolicy.

    @param ssh: paramiko SSH client
    @type  ssh: L{paramiko.SSHClient}
    @param address: SSH IP address
    @type  address: str
    @param port: SSH port
    @type  port: int
    @param username: optional username to include in error message if check fails
    @type  username: str
    @param logger: (optional) logger to record tracebacks to
    @type  logger: logging.Logger
    @return: error message if improperly configured, or None
    @rtype: str
    """
    import paramiko
    try:
        try:
            if os.path.isfile('/etc/ssh/ssh_known_hosts'): #default ubuntu location
                ssh.load_system_host_keys('/etc/ssh/ssh_known_hosts')
        except IOError:
            pass
        ssh.load_system_host_keys() #default user location
    except:
        if logger:
            logger.error(traceback.format_exc())
        # as seen in #767, base64 raises generic Error.
        #
        # A corrupt pycrypto build can also cause this, though
        # the cause of the corrupt builds has been fixed.
        return "cannot load SSH host keys -- your known_hosts file may be corrupt"

    # #1849: paramiko will raise an SSHException with an 'Unknown
    # server' message if the address is not in the known_hosts
    # file. This causes a lot of confusion to users, so we try
    # and diagnose this in advance and offer better guidance

    # - ssh.get_host_keys() does not return the system host keys
    hk = ssh._system_host_keys
    override = os.environ.get('ROSLAUNCH_SSH_UNKNOWN', 0)
    if override == '1':
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    elif hk.lookup(address) is None:
        port_str = user_str = ''
        if port != 22:
            port_str = "-p %s "%port
        if username:
            user_str = username+'@'
        return """%s is not in your SSH known_hosts file.

Please manually:
  ssh %s%s%s

then try roslaunching again.

If you wish to configure roslaunch to automatically recognize unknown
hosts, please set the environment variable ROSLAUNCH_SSH_UNKNOWN=1"""%(address, user_str, port_str, address)
        

class SSHChildROSLaunchProcess(roslaunch.server.ChildROSLaunchProcess):
    """
    Process wrapper for launching and monitoring a child roslaunch process over SSH
    """
    def __init__(self, run_id, name, server_uri, env, machine):
        if machine.ros_root:
            cmd = os.path.join(machine.ros_root, 'bin', 'roslaunch')
        else: # assumes user has mapped onto path
            cmd = 'roslaunch'
        args = [cmd, '-c', name, '-u', server_uri, '--run_id', run_id]
        super(SSHChildROSLaunchProcess, self).__init__(name, args, env)
        self.machine = machine
        self.ssh = self.sshin = self.sshout = self.ssherr = None
        self.started = False
        self.uri = None
        # self.is_dead is a flag set by is_alive that affects whether or not we
        # log errors during a stop(). 
        self.is_dead = False
        
    def _ssh_exec(self, command, env, address, port, username=None, password=None):
        if env:
            env_command = "env "+' '.join(["%s=%s"%(k,v) for (k, v) in env.iteritems()])
            command = "%s %s"%(env_command, command)
        try:
            # as pycrypto 2.0.1 is EOL, disable it's Python 2.6 deprecation warnings
            import warnings
            warnings.filterwarnings("ignore", message="the sha module is deprecated; use the hashlib module instead")
            warnings.filterwarnings("ignore", message="the md5 module is deprecated; use hashlib instead")                                    
            
            import Crypto
        except ImportError, e:
            _logger.error("cannot use SSH: pycrypto is not installed")
            return None, "pycrypto is not installed"
        try:
            import paramiko
        except ImportError, e:
            _logger.error("cannot use SSH: paramiko is not installed")
            return None, "paramiko is not installed"
        #load ssh client and connect
        ssh = paramiko.SSHClient()
        err_msg = ssh_check_known_hosts(ssh, address, port, username=username, logger=_logger)
        
        if not err_msg:
            username_str = '%s@'%username if username else ''
            try:
                if not password: #use SSH agent
                    ssh.connect(address, port, username, timeout=TIMEOUT_SSH_CONNECT)
                else: #use SSH with login/pass
                    ssh.connect(address, port, username, password, timeout=TIMEOUT_SSH_CONNECT)
            except paramiko.BadHostKeyException:
                _logger.error(traceback.format_exc())
                err_msg =  "Unable to verify host key for remote computer[%s:%s]"%(address, port)
            except paramiko.AuthenticationException:
                _logger.error(traceback.format_exc())
                err_msg = "Authentication to remote computer[%s%s:%s] failed.\nA common cause of this error is a missing key in your authorized_keys file."%(username_str, address, port)
            except paramiko.SSHException, e:
                _logger.error(traceback.format_exc())
                if str(e).startswith("Unknown server"):
                    pass
                err_msg = "Unable to establish ssh connection to [%s%s:%s]: %s"%(username_str, address, port, e)
            except socket.error, e:
                # #1824
                if e[0] == 111:
                    err_msg = "network connection refused by [%s:%s]"%(address, port)
                else:
                    err_msg = "network error connecting to [%s:%s]: %s"%(address, port, str(e))
        if err_msg:
            return None, err_msg
        else:
            sshin, sshout, ssherr = ssh.exec_command(command)
            return (ssh, sshin, sshout, ssherr), "executed remotely"

    def start(self):
        """
        Start the remote process. This will create an SSH connection
        to the remote host.
        """
        self.started = False #won't set to True until we are finished
        self.ssh = self.sshin = self.sshout = self.ssherr = None        
        try:
            self.lock.acquire()
            name = self.name
            m = self.machine
            if m.user is not None:
                printlog("remote[%s]: creating ssh connection to %s:%s, user[%s]"%(name, m.address, m.ssh_port, m.user))
            else:
                printlog("remote[%s]: creating ssh connection to %s:%s"%(name, m.address, m.ssh_port))
            _logger.info("remote[%s]: invoking with ssh exec args [%s], env: %s"%(name, ' '.join(self.args), self.env))
            sshvals, msg = self._ssh_exec(' '.join(self.args), self.env, m.address, m.ssh_port, m.user, m.password)
            if sshvals is None:
                printerrlog("remote[%s]: failed to launch on %s:\n\n%s\n\n"%(name, m.name, msg))
                return False
            self.ssh, self.sshin, self.sshout, self.ssherr = sshvals
            printlog("remote[%s]: ssh connection created"%name)
            self.started = True            
            return True
        finally:
            self.lock.release()

    def getapi(self):
        """
        @return: ServerProxy to remote client XMLRPC server
        @rtype: L{ServerProxy}
        """
        if self.uri:
            return xmlrpclib.ServerProxy(self.uri)
        else:
            return None
    
    def is_alive(self):
        """
        @return: True if the process is alive. is_alive needs to be
        called periodically as it drains the SSH buffer
        @rtype: bool
        """
        if self.started and not self.ssh:
            return False
        elif not self.started:
            return True #not started is equivalent to alive in our logic
        s = self.ssherr
        s.channel.settimeout(0)
        try:
            #drain the pipes
            data = s.read(2048)
            if not len(data):
                self.is_dead = True
                return False
            # #2012 il8n: ssh *should* be UTF-8, but often isn't
            # (e.g. Japan)
            data = data.decode('utf-8')
            printerrlog("remote[%s]: %s"%(self.name, data))
        except socket.timeout:
            pass
        except IOError:
            return False
        except UnicodeDecodeError:
            # #2012: soft fail, printing is not essential. This occurs
            # with older machines that don't send utf-8 over ssh
            pass

        s = self.sshout
        s.channel.settimeout(0)
        try:
            #drain the pipes
            #TODO: write to log file
            data = s.read(2048)
            if not len(data):
                self.is_dead = True
                return False
            # data = data.decode('utf-8')
            #print "DATA", data
        except socket.timeout:
            pass
        except IOError:
            return False
        return True

    def stop(self, errors=[]):
        """
        Terminate this process, including the SSH connection.
        """
        try:
            self.lock.acquire()            
            if not self.ssh:
                return

            # call the shutdown API first as closing the SSH connection
            # won't actually kill the process unless it triggers SIGPIPE
            try:
                api = self.getapi()
                if api is not None:
                    #TODO: probably need a timeout on this
                    api.shutdown()
            except socket.error:
                # normal if process is already dead
                address, port = self.machine.address, self.machine.ssh_port
                if not self.is_dead:
                    printerrlog("remote[%s]: unable to contact [%s] to shutdown remote processes!"%(self.name, address))
                else:
                    printlog("remote[%s]: unable to contact [%s] to shutdown cleanly. The remote roslaunch may have exited already."%(self.name, address))
            except:
                # temporary: don't really want to log here as this 
                # may occur during shutdown
                traceback.print_exc()

            _logger.info("remote[%s]: closing ssh connection", self.name)
            self.sshin.close()
            self.sshout.close()
            self.ssherr.close()                        
            self.ssh.close()

            self.sshin  = None
            self.sshout = None
            self.ssherr = None            
            self.ssh = None
            _logger.info("remote[%s]: ssh connection closed", self.name)            
        finally:
            self.lock.release()
