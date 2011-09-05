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



    


class FakeGITClientTest(unittest.TestCase):
    def setUp(self):
        pass
    def tearDown(self):
        pass
    def test_get_url_by_reading(self):
        pass

class GITClientTest(unittest.TestCase):
    def setup_git_repo(directory):

        subprocess.check_call(["git", "init"], cwd=directory)
        subprocess.check_call(["touch", "one"], cwd=directory)
        subprocess.check_call(["git", "add", "one"], cwd=directory)
        subprocess.check_call(["git", "commit", "one", "-m", "\"commit one\""], cwd=directory)
        po = subprocess.Popen(["git", "log", "-n", "1", "--pretty=format:\"%H\""], cwd=directory, stdout=subprocess.PIPE)
        self.hash1 = po.stdout.read().rstrip('"').lstrip('"')
        subprocess.check_call(["git", "branch", "b_two"], cwd=directory)
        subprocess.check_call(["touch", "two"], cwd=directory)
        subprocess.check_call(["git", "add", "two"], cwd=directory)
        subprocess.check_call(["git", "commit", "two", "-m", "\"commit two\""], cwd=directory)
        po = subprocess.Popen(["git", "log", "-n", "1", "--pretty=format:\"%H\""], cwd=directory, stdout=subprocess.PIPE)
        self.hash2 = po.stdout.read().rstrip('"').lstrip('"')

    def setUp(self):
        self.directories = {}
        directory = tempfile.mkdtemp()
        name = "setUp"
        self.directories[name] = directory
        self.readonly_url = "https://github.com/tfoote/ros-get-test.git"
        self.readonly_version = "c55a19422c67b7c83f8c5d6e10b41b1f2711e5a6"
        self.readonly_path = os.path.join(directory, "readonly")
        gitc = git.GITClient(self.readonly_path)
        self.assertTrue(gitc.checkout(self.readonly_url, self.readonly_version))

    def tearDown(self):
        for d in self.directories:
            shutil.rmtree(self.directories[d])

    def test_get_url_by_reading(self):
        gitc = git.GITClient(self.readonly_path)
        self.assertTrue(gitc.path_exists())
        self.assertTrue(gitc.detect_presence())
        self.assertEqual(gitc.get_url(), self.readonly_url)
        self.assertEqual(gitc.get_version(), self.readonly_version)


    def test_get_type_name(self):
        local_path = "/tmp/dummy"
        gitc = git.GITClient(local_path)
        self.assertEqual(gitc.get_vcs_type_name(), 'git')

    def test_checkout(self):
        directory = tempfile.mkdtemp()
        self.directories["checkout_test"] = directory
        local_path = os.path.join(directory, "ros")
        url = "https://github.com/tfoote/ros-get-test.git"
        gitc = git.GITClient(local_path)
        self.assertFalse(gitc.path_exists())
        self.assertFalse(gitc.detect_presence())
        self.assertFalse(gitc.detect_presence())
        self.assertTrue(gitc.checkout(url))
        self.assertTrue(gitc.path_exists())
        self.assertTrue(gitc.detect_presence())
        self.assertEqual(gitc.get_path(), local_path)
        self.assertEqual(gitc.get_url(), url)
        self.assertEqual(gitc.get_branch(), "master")
        self.assertEqual(gitc.get_branch_parent(), "master")
        #self.assertEqual(gitc.get_version(), '-r*')
        

        shutil.rmtree(directory)
        self.directories.pop("checkout_test")

    def test_checkout_specific_version_and_update(self):
        directory = tempfile.mkdtemp()
        subdir = "checkout_specific_version_test"
        self.directories[subdir] = directory
        local_path = os.path.join(directory, "ros")
        url = "https://github.com/tfoote/ros-get-test.git"
        version = "c55a19422c67b7c83f8c5d6e10b41b1f2711e5a6"
        gitc = git.GITClient(local_path)
        self.assertFalse(gitc.path_exists())
        self.assertFalse(gitc.detect_presence())
        self.assertTrue(gitc.checkout(url, version))
        self.assertTrue(gitc.path_exists())
        self.assertTrue(gitc.detect_presence())
        self.assertEqual(gitc.get_path(), local_path)
        self.assertEqual(gitc.get_url(), url)
        self.assertEqual(gitc.get_version(), version)
        
        new_version = '9f7d666bb183f31e8a1443e7b16428ba190c37e3'
        self.assertTrue(gitc.update(new_version))
        self.assertEqual(gitc.get_version(), new_version)
        
        shutil.rmtree(directory)
        self.directories.pop(subdir)

    def test_checkout_specific_branch_and_update(self):
        directory = tempfile.mkdtemp()
        subdir = "checkout_specific_version_test"
        self.directories[subdir] = directory
        local_path = os.path.join(directory, "ros")
        url = "https://github.com/tfoote/ros-get-test.git"
        branch = "master"
        gitc = git.GITClient(local_path)
        self.assertFalse(gitc.path_exists())
        self.assertFalse(gitc.detect_presence())
        self.assertTrue(gitc.checkout(url, branch))
        self.assertTrue(gitc.path_exists())
        self.assertTrue(gitc.detect_presence())
        self.assertEqual(gitc.get_path(), local_path)
        self.assertEqual(gitc.get_url(), url)
        self.assertEqual(gitc.get_branch_parent(), branch)
        
        new_branch = 'master'
        self.assertTrue(gitc.update(new_branch))
        self.assertEqual(gitc.get_branch_parent(), new_branch)
        
        shutil.rmtree(directory)
        self.directories.pop(subdir)


if __name__ == '__main__':
    from ros import rostest
    from roslib import os_detect
    os_detector = os_detect.OSDetect()
    if os_detector.get_name() == "ubuntu" and os_detector.get_version() == "9.04":
        print "jaunty detected, skipping test"
        rostest.unitrun('vcstools', 'test_git', FakeGITClientTest, coverage_packages=['vcstools'])
    else:
        rostest.unitrun('vcstools', 'test_git', GITClientTest, coverage_packages=['vcstools'])
    #setup_git_repo("/tmp/git")
