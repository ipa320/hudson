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
"""
hg vcs support.

New in ROS C-Turtle.
"""

from __future__ import with_statement
import subprocess
import os
import vcs_base
import sys

class HGClient(vcs_base.VCSClientBase):
        
    def __init__(self, path):
        """
        Raise LookupError if hg not detected
        """
        vcs_base.VCSClientBase.__init__(self, path)
        with open(os.devnull, 'w') as fnull:
            try:
                subprocess.call("hg help".split(), stdout=fnull, stderr=fnull)
            except:
                raise LookupError("hg not installed, cannot create a hg vcs client")

    def get_url(self):
        """
        @return: HG URL of the directory path (output of hg paths command), or None if it cannot be determined
        """
        if self.detect_presence():
            output = subprocess.Popen(["hg", "paths", "default"], cwd=self._path, stdout=subprocess.PIPE).communicate()[0]
            return output.rstrip()
        return None

    def detect_presence(self):
        return self.path_exists() and os.path.isdir(os.path.join(self._path, '.hg'))


    def checkout(self, url, version=''):
        if self.path_exists():
            print >>sys.stderr, "Error: cannot checkout into existing directory"
            return False

        # make sure that the parent directory exists for #3497
        base_path = os.path.split(self.get_path())[0]
        try:
            os.makedirs(base_path) 
        except OSError, ex:
            # OSError thrown if directory already exists this is ok
            pass
        
        cmd = "hg clone %s %s"%(url, self._path)
        if not subprocess.call(cmd, shell=True) == 0:
            return False
        cmd = "hg checkout %s"%(version)
        if not subprocess.call(cmd, cwd=self._path, shell=True) == 0:
            return False
        return True

    def update(self, version=''):
        if not self.detect_presence():
            return False
        cmd = "hg pull"
        if not subprocess.call(cmd, cwd=self._path, shell=True) == 0:
            return False
        cmd = "hg checkout %s"%version
        if not subprocess.call(cmd, cwd=self._path, shell=True) == 0:
            return False
        return True

    def get_vcs_type_name(self):
        return 'hg'

    def get_version(self, spec=None):
        """
        @param: (optional) token for identifying version. spec can be
        a whatever is allowed by 'hg log -r', e.g. a tagname, sha-ID,
        revision-number

        @return the current SHA-ID of the repository. Or if spec is
        provided, the SHA-ID of a revision specified by some
        token.
        """
        # detect presence only if we need path for cwd in popen
        if self.detect_presence() and spec != None:
            command = ['hg', 'log', '-r', spec, '.']
            output = subprocess.Popen(command, cwd= self._path, stdout=subprocess.PIPE).communicate()[0]
            if output == None or output.strip() == '' or output.startswith("abort"):
                return None
            else:
                 matches = [l for l in output.split('\n') if l.startswith('changeset: ')]
                 if len(matches) == 1:
                     return matches[0].split(':')[2]+"+"
        else:
            command = ['hg', 'identify', "-i", self._path]
            output = subprocess.Popen(command, stdout=subprocess.PIPE).communicate()[0]
            return output.strip()

class HGConfig(object):
    """
    Configuration information about an SVN repository for a component
    of code. The configuration we maintain is specific to ROS
    toolchain concepts and is not a general notion of SVN configuration.

     * repo_uri: base URI of repo
     * dev_branch: hg branch the code is developed
     * distro_tag: a tag of the latest released code for a specific ROS distribution
     * release_tag: a tag of the code for a specific release
     """

    def __init__(self):
        self.type = 'hg'
        self.repo_uri      = None
        self.anon_repo_uri = None
        self.dev_branch    = None
        self.distro_tag    = None
        self.release_tag   = None

    def __eq__(self, other):
        return self.repo_uri == other.repo_uri and \
            self.anon_repo_uri == other.anon_repo_uri and \
            self.dev_branch == other.dev_branch and \
            self.release_tag == other.release_tag and \
            self.distro_tag == other.distro_tag
