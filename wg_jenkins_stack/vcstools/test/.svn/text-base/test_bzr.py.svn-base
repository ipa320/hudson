#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

from vcstools import svn, bzr, git, hg

class BZRClientTest(unittest.TestCase):

    def setUp(self):
        self.directories = {}
        directory = tempfile.mkdtemp()
        name = "setUp"
        self.directories[name] = directory
        self.readonly_url = "http://bazaar.launchpad.net/~tully.foote/+junk/ros-vcstools-test/"
        self.readonly_version = "1"
        self.readonly_path = os.path.join(directory, "readonly")
        bzrc = bzr.BZRClient(self.readonly_path)
        self.assertTrue(bzrc.checkout(self.readonly_url, self.readonly_version))

    def tearDown(self):
        for d in self.directories:
            shutil.rmtree(self.directories[d])

    def test_get_url_by_reading(self):
        bzrc = bzr.BZRClient(self.readonly_path)
        self.assertTrue(bzrc.path_exists())
        self.assertTrue(bzrc.detect_presence())
        self.assertEqual(bzrc.get_url(), self.readonly_url)
        self.assertEqual(bzrc.get_version(), self.readonly_version)


    def test_get_type_name(self):
        local_path = "/tmp/dummy"
        bzrc = bzr.BZRClient(local_path)
        self.assertEqual(bzrc.get_vcs_type_name(), 'bzr')

    def test_checkout(self):
        directory = tempfile.mkdtemp()
        self.directories["checkout_test"] = directory
        local_path = os.path.join(directory, "ros")
        url = "http://bazaar.launchpad.net/~tully.foote/+junk/ros-vcstools-test/"
        bzrc = bzr.BZRClient(local_path)
        self.assertFalse(bzrc.path_exists())
        self.assertFalse(bzrc.detect_presence())
        self.assertFalse(bzrc.detect_presence())
        self.assertTrue(bzrc.checkout(url))
        self.assertTrue(bzrc.path_exists())
        self.assertTrue(bzrc.detect_presence())
        self.assertEqual(bzrc.get_path(), local_path)
        self.assertEqual(bzrc.get_url(), url)

        #self.assertEqual(bzrc.get_version(), '-r*')
        

        shutil.rmtree(directory)
        self.directories.pop("checkout_test")

    def test_checkout_specific_version_and_update(self):
        directory = tempfile.mkdtemp()
        subdir = "checkout_specific_version_test"
        self.directories[subdir] = directory
        local_path = os.path.join(directory, "ros")
        url = "http://bazaar.launchpad.net/~tully.foote/+junk/ros-vcstools-test/"
        version = "1"
        bzrc = bzr.BZRClient(local_path)
        self.assertFalse(bzrc.path_exists())
        self.assertFalse(bzrc.detect_presence())
        self.assertFalse(bzrc.detect_presence())
        self.assertTrue(bzrc.checkout(url, version))
        self.assertTrue(bzrc.path_exists())
        self.assertTrue(bzrc.detect_presence())
        self.assertEqual(bzrc.get_path(), local_path)
        self.assertEqual(bzrc.get_url(), url)
        self.assertEqual(bzrc.get_version(), version)
        
        new_version = '2'
        self.assertTrue(bzrc.update(new_version))
        self.assertEqual(bzrc.get_version(), new_version)
        
        shutil.rmtree(directory)
        self.directories.pop(subdir)

class FakeBZRClientTest(unittest.TestCase):


    def test_get_url_by_reading(self):
        self.assertTrue("Test not running on Jaunty")



if __name__ == '__main__':
    from ros import rostest
    from roslib import os_detect
    os_detector = os_detect.OSDetect()
    if os_detector.get_name() == "ubuntu" and os_detector.get_version() == "9.04":
        print "jaunty detected, skipping test"
        rostest.unitrun('vcstools', 'test_vcs', FakeBZRClientTest, coverage_packages=['vcstools'])  
    else:
        rostest.unitrun('vcstools', 'test_vcs', BZRClientTest, coverage_packages=['vcstools'])  

