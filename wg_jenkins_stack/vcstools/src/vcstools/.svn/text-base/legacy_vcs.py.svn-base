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
# Revision $Id: vcs.py 8428 2010-02-20 10:26:42Z kwc $
# $Author: kwc $

"""
Utilities for dealing with Version-control systems

New in ROS C-Turtle.
"""

from __future__ import with_statement

import os

def get_svn_url(dir_path):
    """
    @return: SVN URL of the directory path (output of svn info command), or None if it cannot be determined
    """
    if os.path.isdir(os.path.join(dir_path, '.svn')):
        import subprocess
        output = subprocess.Popen(['svn', 'info', dir_path], stdout=subprocess.PIPE).communicate()[0]
        matches = [l for l in output.split('\n') if l.startswith('URL: ')]
        if matches:
            return matches[0][5:]
    return None
    
def guess_vcs_uri(dir_path):
    """
    Guess the repository URI of the version-controlled directory path
    @param path: directry path
    @type  path: str
    @return: version control system and URI, e.g. 'svn', 'http://code.ros.org/svn/ros'. Return None, None if VCS cannot be determined.
    @rtype: str, str
    """
    repo = None, None
    try:
        if os.path.isdir(os.path.join(dir_path, '.svn')):
            # shell out to svn info and parse the output
            import subprocess
            output = subprocess.Popen(['svn', 'info', dir_path], stdout=subprocess.PIPE).communicate()[0]
            matches = [l for l in output.split('\n') if l.startswith('Repository Root: ')]
            if matches:
                repo = ('svn', matches[0][17:]) 
        else:
            # check parent directories for the .git config directory
            dir_path = os.path.abspath(dir_path)
            while dir_path and dir_path != os.path.dirname(dir_path) and repo == (None, None):
                if os.path.isdir(os.path.join(dir_path, '.git')):
                    with open(os.path.join(dir_path, '.git', 'config')) as config:
                        in_section = False
                        for l in config.readlines():
                            if l.strip() == '[remote "origin"]':
                                in_section = True
                            elif in_section and l.startswith('\turl = '):
                                repo = 'git', l[7:].strip()
                                break
                else:
                    dir_path = os.path.dirname(dir_path)
    except Exception, e:
        pass
    return repo
    

def svn_url_exists(url):
    """
    @return: True if SVN url points to an existing resource
    """
    import subprocess
    try:
        p = subprocess.Popen(['svn', 'info', url], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        p.wait()
        return p.returncode == 0
    except:
        return False

