cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_pyunit(test/test_hg.py TIMEOUT 500)
rosbuild_add_pyunit(test/test_git.py TIMEOUT 500)
rosbuild_add_pyunit(test/test_bzr.py TIMEOUT 500)
rosbuild_add_pyunit(test/test_svn.py TIMEOUT 500)
rosbuild_add_pyunit(test/test_tar.py TIMEOUT 500)
rosbuild_add_pyunit(test/test_vcstools_legacy_vcs.py)
