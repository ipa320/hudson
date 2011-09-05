#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

import roslib; roslib.load_manifest('vcstools')

import os
import stat
import struct
import sys
import unittest
import subprocess
import tempfile
import urllib
import shutil

from vcstools import tar

class TARClientTest(unittest.TestCase):

    def setUp(self):
        self.directories = {}
        directory = tempfile.mkdtemp()
        name = "setUp"
        self.directories[name] = directory
        self.readonly_url = "https://code.ros.org/svn/release/download/stacks/exploration/exploration-0.3.0/exploration-0.3.0.tar.bz2"
        self.readonly_version = "exploration-0.3.0"
        self.readonly_path = os.path.join(directory, "readonly")
        tarc = tar.TARClient(self.readonly_path)
        self.assertTrue(tarc.checkout(self.readonly_url, self.readonly_version))

    def tearDown(self):
        for d in self.directories:
            shutil.rmtree(self.directories[d])

    def test_get_url_by_reading(self):
        tarc = tar.TARClient(self.readonly_path)
        self.assertTrue(tarc.path_exists())
        self.assertTrue(tarc.detect_presence())
        self.assertEqual(tarc.get_url(), self.readonly_url)
        #self.assertEqual(tarc.get_version(), self.readonly_version)


    def test_get_type_name(self):
        local_path = "/tmp/dummy"
        tarc = tar.TARClient(local_path)
        self.assertEqual(tarc.get_vcs_type_name(), 'tar')

    def test_checkout(self):
        directory = tempfile.mkdtemp()
        self.directories["checkout_test"] = directory
        local_path = os.path.join(directory, "exploration")
        url = "https://code.ros.org/svn/release/download/stacks/exploration/exploration-0.3.0/exploration-0.3.0.tar.bz2"
        tarc = tar.TARClient(local_path)
        self.assertFalse(tarc.path_exists())
        self.assertFalse(tarc.detect_presence())
        self.assertFalse(tarc.detect_presence())
        self.assertTrue(tarc.checkout(url))
        self.assertTrue(tarc.path_exists())
        self.assertTrue(tarc.detect_presence())
        self.assertEqual(tarc.get_path(), local_path)
        self.assertEqual(tarc.get_url(), url)

        #self.assertEqual(tarc.get_version(), '-r*')
        

        shutil.rmtree(directory)
        self.directories.pop("checkout_test")






if __name__ == '__main__':
    from ros import rostest
    rostest.unitrun('vcstools', 'test_vcs', TARClientTest, coverage_packages=['vcstools'])  
