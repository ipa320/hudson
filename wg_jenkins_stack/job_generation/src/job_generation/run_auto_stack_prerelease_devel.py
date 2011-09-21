#!/usr/bin/python

STACK_DIR = 'stack_overlay'
DEPENDS_DIR = 'depends_overlay'
#DEPENDS_ON_DIR = 'depends_on_overlay'


import roslib; roslib.load_manifest("job_generation")
from roslib import stack_manifest
from jobs_common import *
import rosdistro
from apt_parser import parse_apt
import subprocess
import os
import sys


def remove(list1, list2):
    for l in list2:
        if l in list1:
            list1.remove(l)


def main():
    # global try
    try:
        # parse command line options
        (options, args) = get_options(['stack', 'rosdistro', 'githubuser', 'email'], ['repeat', 'source-only'])
        if not options:
            return -1

        # set environment
        env = get_environment()
        env['ROS_PACKAGE_PATH'] = '%s:%s:/opt/ros/%s/stacks'%(env['INSTALL_DIR']+'/'+STACK_DIR,
                                                                 env['INSTALL_DIR']+'/'+DEPENDS_DIR,
                                                                 options.rosdistro) # env['INSTALL_DIR']+'/'+DEPENDS_ON_DIR,

        if 'ros' in options.stack:
            env['ROS_ROOT'] = env['INSTALL_DIR']+'/'+STACK_DIR+'/ros'
            print "We're building ROS, so setting the ROS_ROOT to %s"%(env['ROS_ROOT'])
        else:
            env['ROS_ROOT'] = '/opt/ros/%s/ros'%options.rosdistro
        env['PYTHONPATH'] = env['ROS_ROOT']+'/core/roslib/src'
        env['PATH'] = '/opt/ros/%s/ros/bin:%s'%(options.rosdistro, os.environ['PATH'])
        print "HOME: %s"%(env['HOME'])


        # Parse distro file
        rosdistro_obj = rosdistro.Distro(get_rosdistro_file(options.rosdistro))
        print 'Operating on ROS distro %s'%rosdistro_obj.release_name


        # Install the stacks to test from source
        print 'Installing the stacks to test from source'
        rosinstall = ''
        for stack in options.stack:
            if not stack_forked(options.githubuser, stack):
                print "Stack %s is not forked for user %s" %(stack, options.githubuser)
                print "Using 'ipa320' stack instead"
                options.githubuser = "ipa320"
            if stack in FHG_STACKS_PUBLIC: # create rosinstall file for public stacks
                rosinstall += '- git: {local-name: %s, uri: "git://github.com/%s/%s.git", branch-name: master}\n'%(stack, options.githubuser, stack)
            elif stack in FHG_STACKS_PRIVATE: # clone private stacks
                call('git clone git@github.com:%s/%s.git %s'%(options.githubuser, stack, STACK_DIR), env, 'Clone private stack [%s] to test'%(stack))
            else:
                rosinstall += stack_to_rosinstall(rosdistro_obj.stacks[stack], 'devel')

        if rosinstall != '': # call rosinstall command
            rosinstall_file = '%s.rosinstall'%STACK_DIR
            print 'Generating rosinstall file [%s]'%(rosinstall_file)
            print 'Contents:\n\n'+rosinstall+'\n\n'
            with open(rosinstall_file, 'w') as f:
                f.write(rosinstall)
            print 'rosinstall file [%s] generated'%(rosinstall_file)
            print STACK_DIR
            print options.rosdistro
            call('rosinstall --rosdep-yes %s /opt/ros/%s %s'%(STACK_DIR, options.rosdistro, rosinstall_file), env,
                 'Install the stacks to test from source.')
            
        
        # get all stack dependencies of stacks we're testing

        depends_all = {"public" : [], "private" : [], "other" : []}
        for stack in options.stack:
#            stack_xml = '%s/%s/stack.xml'%(STACK_DIR, stack)
#            call('ls %s'%stack_xml, env, 'Checking if stack %s contains "stack.xml" file'%stack)
#            with open(stack_xml) as stack_file:
#                depends_one = [str(d) for d in stack_manifest.parse(stack_file.read()).depends]  # convert to list
            depends_one = get_depends_one(stack, options.githubuser)
            print 'Dependencies of stack %s: %s'%(stack, str(depends_one))
            for d in depends_one:
                if not d in options.stack and not d in depends_all:
                    if stack == "cob3_intern":
                        if not d in COB3_INTERN_STACKS_DEPS and not d in COB3_INTERN_STACKS:
                            print 'Adding dependencies of stack %s'%d
                            get_depends_all(d, depends_all, options.githubuser)
                            print 'Resulting total dependencies of all stacks that get tested: %s'%str(depends_all)
                    else:
                        print 'Adding dependencies of stack %s'%d
                        get_depends_all(d, depends_all, options.githubuser)
                        print 'Resulting total dependencies of all stacks that get tested: %s'%str(depends_all) 
        

        if len(depends_all["private"]) > 0:
            print 'Cloning private github fork(s)'
            downloaded = False
            for stack in depends_all["private"]:
                if stack in COB3_INTERN_STACKS:
                    if not downloaded:
                        if not stack_forked(options.githubuser, "cob3_intern", "/blob/master/%s/Makefile"%stack):
                            options.githubuser = "ipa320"
                        call('git clone git@github.com:%s/cob3_intern.git %s'%(options.githubuser, "/tmp/cob3_intern"), env, 'Clone private stack cob3_intern')
                        downloaded = True
                        
                    call('mv /tmp/cob3_intern/%s %s'%(stack, DEPENDS_DIR), env, 'Move required stack %s to %s'%(stack, DEPENDS_DIR))
                    
                else:
                    if not stack_forked(options.githubuser, stack):
                        options.githubuser = "ipa320"
                    call('git clone git@github.com:%s/%s.git %s'%(options.githubuser, stack, DEPENDS_DIR), env, 'Clone private stack [%s] to test'%(stack))
                    

        if len(depends_all["public"]) > 0:
            for stack in depends_all["public"]:
                if not stack_forked(options.githubuser, stack):
                    options.githubuser = "ipa320"
                rosinstall += '- git: {local-name: %s, uri: "git://github.com/%s/%s.git", branch-name: master}\n'%(stack, options.githubuser, stack)
            
            print 'Installing stack dependencies from public github fork'
            rosinstall_file = '%s.rosinstall'%DEPENDS_DIR
            with open(rosinstall_file, 'w') as f:
                f.write(rosinstall)
            call('rosinstall --rosdep-yes %s /opt/ros/%s %s'%(DEPENDS_DIR, options.rosdistro, rosinstall_file), env,
                 'Install the stack dependencies from source.')
     
        if len(depends_all["other"]) > 0:
            # Install Debian packages of stack dependencies
            print 'Installing debian packages of "%s" dependencies: %s'%(stack, str(depends_all["other"]))
            call('sudo apt-get update', env)
            call('sudo apt-get install %s --yes'%(stacks_to_debs(depends_all["other"], options.rosdistro)), env)
 
        depends_no = 0
        for i in iter(depends_all): depends_no += len(depends_all[i])
        if depends_no == 0:
            print 'Stack(s) %s do(es) not have any dependencies, not installing anything now'%str(options.stack)
            
        # Install system dependencies of stacks we're testing
        print "Installing system dependencies of stacks we're testing"
        call('rosmake rosdep', env)
        for stack in options.stack:
            if stack == "cob3_intern":
                call('make -f /tmp/install_dir/%s/Makefile ros-install-%s'%(STACK_DIR, options.rosdistro), env, 'ros-install')
                call('make -f /tmp/install_dir/%s/Makefile ros-skip-blacklist'%STACK_DIR, env, 'ros-skip-blacklist')
            else:
                call('rosdep install -y %s'%stack, env,
                     'Install system dependencies of stack %s'%stack)
                 
        # Run hudson helper for stacks only
        print 'Running Hudson Helper'
        res = 0
        for r in range(0, int(options.repeat)+1):
            env['ROS_TEST_RESULTS_DIR'] = env['ROS_TEST_RESULTS_DIR'] + '/' + STACK_DIR + '_run_' + str(r)
            helper = subprocess.Popen(('./hudson_helper_fhg.py --dir-test %s --email %s build'%(STACK_DIR, options.email)).split(' '), env=env)
            helper.communicate()
            if helper.returncode != 0:
                res = helper.returncode
        if res != 0:
            return res
        
        
        # parse debian repository configuration file to get stack dependencies
        arch = 'i386'
        if '64' in call('uname -mrs', env):
            arch = 'amd64'
        ubuntudistro = call('lsb_release -a', env).split('Codename:')[1].strip()
        print "Parsing apt repository configuration file to get stack dependencies, for %s machine running %s"%(arch, ubuntudistro)
        apt_deps = parse_apt(ubuntudistro, arch, options.rosdistro)
        # all stacks that depends on the tested stacks, excluding the tested stacks.
        depends_on_all = apt_deps.depends_on_all(options.stack)
        remove(depends_on_all, options.stack)
        # all stack dependencies of above stack list, except for the test stack dependencies
        depends_all_depends_on_all = apt_deps.depends_all(depends_on_all)
        remove(depends_all_depends_on_all, options.stack)
        remove(depends_all_depends_on_all, depends_all)
        
    # global except
    except Exception, ex:
        print "Global exception caught. Generating email"
        generate_email("%s. Check the console output for test failure details."%ex, env)
        traceback.print_exc(file=sys.stdout)
        raise ex
        
        

if __name__ == '__main__':
    try:
        res = main()
        sys.exit( res )
    except Exception, ex:
        sys.exit(-1)
