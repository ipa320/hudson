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
import traceback


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
        env['ROS_PARALLEL_JOBS'] = '-j4'
        print "HOME: %s"%(env['HOME'])


        # Parse distro file
        rosdistro_obj = rosdistro.Distro(get_rosdistro_file(options.rosdistro))
        print 'Operating on ROS distro %s'%rosdistro_obj.release_name


        # Install the stacks to test from source
        print '\n==================================================================================='
        print 'Installing %s to test from source\n'%str(options.stack)
        rosinstall = ''
        for stack in options.stack:
            if not stack_forked(options.githubuser, stack):
                raise Exception('  Stack %s is not forked for user %s! Fork stack on github.com to run job'%(stack, options.githubuser))
                
            rosinstall += stack_origin(rosdistro_obj, rosinstall, stack, options.githubuser, STACK_DIR, env)
            
        if rosinstall != '': # call rosinstall command
            print '\n==================================================================================='
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
        rosinstall = ''
        depends_all = {"public" : [], "private" : [], "other" : []}
        for stack in options.stack:
            print '\n==================================================================================='
            print 'Calculating all stack dependencies of %s\n'%stack
            depends_one = get_depends_one(stack, options.githubuser)

            for d in depends_one:
                depends_all_list = []
                [[depends_all_list.append(value) for value in valuelist] for valuelist in depends_all.itervalues()]
                if not d in options.stack and not d in depends_all_list:
                    get_depends_all(d, depends_all, options.githubuser, 1)
        
        print '\n==================================================================================='
        print 'Dependencies of %s:'%str(options.stack)
        print "  Private IPA stacks:"
        print "    ", str(depends_all["private"])
        print "  Public IPA stacks:"
        print "    ", str(depends_all["public"])
        print "  None IPA stacks:"
        print "    ", str(depends_all["other"])

        if len(depends_all["private"]) > 0:#TODO released private stack???
            print '\n==================================================================================='
            print 'Cloning private github fork(s)\n'
            for stack in depends_all["private"]:
                rosinstall += stack_origin(rosdistro_obj, rosinstall, stack, options.githubuser, DEPENDS_DIR, env)
                    

        if len(depends_all["public"]) > 0:
            print '\n==================================================================================='
            print 'Installing stack dependencies from public github fork\n'
            for stack in depends_all["public"]:
                rosinstall += stack_origin(rosdistro_obj, rosinstall, stack, options.githubuser, DEPENDS_DIR, env)
            
            rosinstall_file = '%s.rosinstall'%DEPENDS_DIR
            with open(rosinstall_file, 'w') as f:
                f.write(rosinstall)
            call('rosinstall --rosdep-yes %s /opt/ros/%s %s'%(DEPENDS_DIR, options.rosdistro, rosinstall_file), env,
                 'Install the stack dependencies from source.')
     
        if len(depends_all["other"]) > 0:
            # Install Debian packages of stack dependencies
            print '\n==================================================================================='
            print 'Installing debian packages of "%s" dependencies:\n\n%s'%(str(options.stack), str(depends_all["other"]))
            call('sudo apt-get update', env)
            call('sudo apt-get install %s --yes'%(stacks_to_debs(depends_all["other"], options.rosdistro)), env)
 
        depends_no = 0
        for i in iter(depends_all): depends_no += len(depends_all[i])
        if depends_no == 0:
            print 'Stack(s) %s do(es) not have any dependencies, not installing anything now'%str(options.stack)
        
      
        # Install system dependencies of stacks we're testing
        print '\n==================================================================================='
        print "Installing system dependencies of stacks we're testing"
        call('rosmake rosdep', env)
        for stack in options.stack:
            call('rosdep install -y %s'%stack, env,
                 'Install system dependencies of stack %s'%stack)
                 
        # Run hudson helper for stacks only
        print '\n==================================================================================='
        print 'Running Hudson Helper\n'
        res = 0
        for r in range(0, int(options.repeat)+1):
            env['ROS_TEST_RESULTS_DIR'] = env['ROS_TEST_RESULTS_DIR'] + '/' + STACK_DIR + '_run_' + str(r)
            print '\nHUDSON HELPER CALL:'
            helper = subprocess.Popen(('./ros_release/hudson/src/hudson_helper_fhg.py --dir-test %s --email %s build'%(STACK_DIR, options.email)).split(' '), env=env)
            helper.communicate()
            if helper.returncode != 0:
                res = helper.returncode
        if res != 0:
            return res
        
        build_failures = os.path.join(os.environ['WORKSPACE'], 'build_output', 'buildfailures.txt')
        build_failures_context = os.path.join(os.environ['WORKSPACE'], 'build_output', 'buildfailures-with-context.txt')
        test_failures = os.path.join(os.environ['WORKSPACE'], 'test_output', 'testfailures.txt')
        body = ''
        if os.path.exists(build_failures):
            body += open(build_failures).read()
            body += '\n\n'
        if os.path.exists(test_failures):
            body += open(test_failures).read()
            body += '\n\n'
        if os.path.exists(build_failures_context):
            body += open(build_failures_context).read()
            body += '\n\n'
        print "\n******************************************************************"
        print "***                   BUILD AND TEST FAILURES                  ***"
        print "******************************************************************\n"
        print body
        print "******************************************************************"
        
#no need to test debian package dependencies as long as no debian package exist
#        # parse debian repository configuration file to get stack dependencies
#        arch = 'i386'
#        if '64' in call('uname -mrs', env):
#            arch = 'amd64'
#        ubuntudistro = call('lsb_release -a', env).split('Codename:')[1].strip()
#        print "Parsing apt repository configuration file to get stack dependencies, for %s machine running %s"%(arch, ubuntudistro)
#        apt_deps = parse_apt(ubuntudistro, arch, options.rosdistro)
#        # all stacks that depends on the tested stacks, excluding the tested stacks.
#        depends_on_all = apt_deps.depends_on_all(options.stack)
#        remove(depends_on_all, options.stack)
#        # all stack dependencies of above stack list, except for the test stack dependencies
#        depends_all_depends_on_all = apt_deps.depends_all(depends_on_all)
#        remove(depends_all_depends_on_all, options.stack)
#        remove(depends_all_depends_on_all, depends_all)
        
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
