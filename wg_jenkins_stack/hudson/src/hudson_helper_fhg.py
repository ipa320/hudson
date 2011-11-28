#!/usr/bin/env python
from __future__ import with_statement

from optparse import OptionParser
import sys, os
import shutil
import urllib
import urllib2
from subprocess import Popen, PIPE, call, check_call, CalledProcessError
import stat

NAME='hudson_helper'

print >> sys.stderr, '[%s] release mode'%(NAME)

EMAIL_FROM_ADDR = 'hudson@ipa.fhg.de'

dummy_test_results_simple = """<?xml version="1.0" encoding="utf-8"?><testsuite name="dummy.TEST-test_dummy" tests="1" errors="0" failures="0" time="0.037">  <testcase classname="dummy.TEST-test_dummy.NotTested" name="dummy.test_dummy/NotTested" time="0.0">
  </testcase>  <system-out><![CDATA[]]></system-out>  <system-err><![CDATA[]]></system-err></testsuite>"""

# Temporary paste-in from roslib
def list_pkgs(pkg_dirs):
    packages = []
    for pkgRoot in pkg_dirs:
        for dir, dirs, files in os.walk(pkgRoot, topdown=True):
            if 'manifest.xml' in files:
                package = os.path.basename(dir)
                if package not in packages:
                  packages.append(package)
                del dirs[:]
            elif 'rospack_nosubdirs' in files:
                del dirs[:]
            #small optimization
            elif '.svn' in dirs:
                dirs.remove('.svn')
            elif '.git' in dirs:
                dirs.remove('.git')
    return packages
    
def send_mail(from_addr, to_addrs, subject, text):
    import smtplib
    from email.mime.text import MIMEText

    msg = MIMEText(text)
    msg['From'] = from_addr
    msg['To'] = to_addrs
    msg['Subject'] = subject

    s = smtplib.SMTP('io.ipa.fhg.de')
    print >> sys.stderr, '[%s] Sending mail to %s'%(NAME,to_addrs)
    s.sendmail(msg['From'], [msg['To']], msg.as_string())
    s.quit()

class HudsonHelper:

    def __init__(self, argv):
        self._parse_args(argv)

    def _parse_args(self, argv):
        parser = OptionParser(usage="usage: %prog [options] <cmd>", prog=NAME)
        parser.add_option("--dir-test", action="append", nargs=1,
                          dest="dirs_test", default=[],
                          help="directory to append to RPP, and test in")
        parser.add_option("--email", action="store",
                          dest="email", default=None,
                          help="email address to send results to")
        parser.add_option("--threads", action="store",
                          dest="threads", default=0,
                          help="Build up to N packages in parallel")
        (options, args) = parser.parse_args(argv)
    
        if len(args) < 2:
            parser.error('must specify command')
        self.cmd = args[1]
    
        if len(options.dirs_test) == 0:
            parser.error("nothing to do; must specify --dir-test")
        
        self.rosmake_args_threads = []
        if otions.threads != 0:
            self.rosmake_args_threads.append('--threads=%s'%options.threads)

        self.email = options.email
        # NOTE: the following addition puts directories specified by --dir-test
        # ahead of those specified by --dir.  This is correct for ondemand test
        # builds of stack/trunk against */latest, but will not be right in
        # general.
        self.dirs = options.dirs_test
        self.dirs_test = options.dirs_test

    def main(self):
        stderrs = []
        self.build()
        
    def build(self):
        # Hudson sets the WORKSPACE env var
        workspace = os.environ['WORKSPACE']

        # If ROS_ROOT was already set in the environment, take that value
        # for the build.
        if 'ROS_ROOT' in os.environ:
            ros_root = os.environ['ROS_ROOT']
        else:
            ros_root = os.path.join(workspace, 'ros')

        # if ROS_TEST_RESULTS_DIR was already set in the environment, 
        # take that value for the build
        if 'ROS_TEST_RESULTS_DIR' in os.environ:
          ros_test_results_dir = os.environ['ROS_TEST_RESULTS_DIR']
          print '1. Test results dir is set to %s in environment'%ros_test_results_dir
        else:
          ros_test_results_dir = os.path.join(ros_root, 'test', 'test_results')
          print '1. Test results dir is not set in environment, using ROS_ROOT/test/test_results'

        # Blow away old ~/.ros content. self.dotrosname is used at the end
        # of this method, for tarring up the result of this run.
#        self.dotrosname = os.path.join(workspace, '.ros')
#        if os.path.isdir(self.dotrosname):
#            try:
#                shutil.rmtree(self.dotrosname)
#            except OSError, e:
#        	# Ignore this; it's usually a stale NFS handle.
#                pass
#        elif os.path.isfile(self.dotrosname):
#            os.unlink(self.dotrosname)
#        if not os.path.isdir(self.dotrosname):
#            os.mkdir(self.dotrosname)

        self.rosmake_path = os.path.join(ros_root, 'bin', 'rosmake')

        if 'PATH' in os.environ:
            path = ':'.join([os.path.join(ros_root, 'bin'), os.environ['PATH']])
        else:
            path = os.path.join(ros_root, 'bin')

        if 'PYTHONPATH' in os.environ:
            pythonpath = ':'.join([os.environ['PYTHONPATH'], os.path.join(ros_root, 'core', 'roslib', 'src')])
        else:
            pythonpath = os.path.join(ros_root, 'core', 'roslib', 'src')
        local_paths = []
#        for r in self.repos:
#            local_paths.append(os.path.abspath(r[0]))
        for d in self.dirs:
            if not os.path.samefile(os.path.abspath(d), ros_root):
                local_paths.append(os.path.abspath(d))
        if 'ROS_PACKAGE_PATH' in os.environ:
            # Prepend directories given on the command-line, on the
            # assumption that they are intended as overlays atop whatever
            # was in the environment already.
            ros_package_path = ':'.join(local_paths + [os.environ['ROS_PACKAGE_PATH']])
        else:
            ros_package_path = ':'.join(local_paths)

        env_vars = os.environ.copy()
        # The JAVA_HOME setting is specific to Ubuntu.
        env_vars.update({'ROS_ROOT' : ros_root,
                         'ROS_PACKAGE_PATH' : ros_package_path,
                         'ROSDEP_YES' : '1',
                         'ROS_TEST_RESULTS_DIR' : ros_test_results_dir,
                         'PATH' : path,
                         'PYTHONPATH' : pythonpath,
                         'ROS_MASTER_URI' : 'http://localhost:11311',
                         'ROBOT' : 'sim',
                         'JAVA_HOME' : '/usr/lib/jvm/java-6-openjdk/',
                         'DISPLAY' : ':0.0'})

        if 'SVN_REVISION' in env_vars:
            del env_vars['SVN_REVISION']

        failure = False
        test_failure = False

        ###########################################
        # poor man's check for capabilities in rosmake
        build_cmd = [self.rosmake_path, '-h']
        rosmake_help = Popen(build_cmd, cwd=ros_root, env=env_vars, stdout=PIPE, stdin=sys.stdin, stderr=PIPE).communicate()[0]
        self.extra_rosmake_args = []
        if 'status-rate' in rosmake_help:
            print >> sys.stderr, "[%s] Detected new version of rosmake"%(NAME)
            self.extra_rosmake_args.append('--status-rate=0')
        else:
            print >> sys.stderr, "[%s] Detected old version of rosmake"%(NAME)

        ###########################################
        # ugly way to pre-cache SVN server certificates, to avoid failed
        # checkouts during the build
        build_cmd = [os.path.join(ros_root, 'tools', 'rospack', 'roscachesvncert')]
        check_call(build_cmd, cwd=ros_root, env=env_vars, stdout=sys.stdout, stdin=sys.stdin, stderr=sys.stderr)
        

        ###########################################
        # bootstrap build in ROS_ROOT
        build_cmd = [self.rosmake_path, '--rosdep-install', '--rosdep-yes'] + self.extra_rosmake_args ['rospack'] + self.rosmake_args_threads
        try:
            print >> sys.stderr, '[%s] %s'%(NAME,build_cmd)
            check_call(build_cmd, cwd=ros_root, env=env_vars, stdout=sys.stdout, stdin=sys.stdin, stderr=sys.stderr)
        except (CalledProcessError, OSError), e:
            failure = True
            print >> sys.stderr, '[%s] Error in bootstrap build step:%s'%(NAME,e)
            #self.post_build(failure, test_failure, workspace)


        ###########################################
        # rosmake build everything
        output_dir = os.path.join(os.environ['PWD'], 'build_output')
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
        # Create dummy result files, to avoid Groovy stack traces in
        # emails, #3214. The files will be closed because we don't save
        # the result from open().
        os.makedirs(output_dir)
        open(os.path.join(output_dir, 'buildfailures.txt'), 'w')
        open(os.path.join(output_dir, 'buildfailures-with-context.txt'), 'w')

        build_cmd = [self.rosmake_path, '-Vr', '--rosdep-install', '--rosdep-yes', '--profile', '--skip-blacklist', '--output=%s'%output_dir] + self.extra_rosmake_args + ['-a'] + self.rosmake_args_threads

        try:
            print >> sys.stderr, '[%s] %s'%(NAME,build_cmd)
            check_call(build_cmd, cwd=ros_root, env=env_vars, stdout=sys.stdout, stdin=sys.stdin, stderr=sys.stderr)
        except (CalledProcessError, OSError), e:
            failure = True
            print >> sys.stderr, '[%s] Error in build step:%s'%(NAME,e)
            #self.post_build(failure, test_failure, workspace)

        ###########################################
        # rosmake test everything
        if os.path.exists(ros_test_results_dir) and not os.path.isdir(ros_test_results_dir):
            os.unlink(ros_test_results_dir)
            os.makedirs(ros_test_results_dir)
        elif os.path.isdir(ros_test_results_dir):
            ros_test_results_dir_dirs = os.listdir(ros_test_results_dir)
            for d in ros_test_results_dir_dirs:
                if d != '.svn' and os.path.isdir(os.path.join(ros_test_results_dir,d)):
                    shutil.rmtree(os.path.join(ros_test_results_dir,d))
        else:
            os.makedirs(ros_test_results_dir)

        output_dir = os.path.join(os.environ['PWD'], 'test_output')
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)

        # Create dummy result files, to avoid Groovy stack traces in
        # emails, #3214. The files will be closed because we don't save
        # the result from open().
        os.makedirs(output_dir)
        open(os.path.join(output_dir, 'testfailures.txt'), 'w')

        build_cmd = [self.rosmake_path, '-Vr', '--profile', '--test-only', '--skip-blacklist', '--output=%s'%output_dir] + self.extra_rosmake_args + self.rosmake_args_threads

        local_test_paths = []
        for d in self.dirs_test:
            local_test_paths.append(os.path.abspath(d))

        test_pkgs = list_pkgs(local_test_paths)
        build_cmd += test_pkgs

        try:
            print >> sys.stderr, '[%s] %s'%(NAME,build_cmd)
            check_call(build_cmd, cwd=ros_root, env=env_vars, stdout=sys.stdout, stdin=sys.stdin, stderr=sys.stderr)
        except (CalledProcessError, OSError), e:
            print >> sys.stderr, '[%s] Error in test step:%s'%(NAME, e)
            test_failure = True

        #rostest_dir = roslib.packages.get_pkg_dir('rostest')
            
        # Backwards compatability hack for boxturtle and cturtle
        rostest_dir = os.path.join(ros_root, 'test', 'rostest')
        cleanunit_rostest = os.path.join(rostest_dir, 'bin', 'cleanunit')
        if os.path.exists(cleanunit_rostest):
          cleanunit_cmd = [cleanunit_rostest]
        else: # end hack
          cleanunit_cmd = [os.path.join(ros_root, 'tools', 'rosunit', 'scripts', 'clean_junit_xml.py')]

        print >> sys.stderr, '[%s] %s'%(NAME,cleanunit_cmd)
        check_call(cleanunit_cmd, cwd=ros_root, env=env_vars, stdout=sys.stdout, stdin=sys.stdin, stderr=sys.stderr)

        # insert dummy xml file
        output_dir = os.path.join(ros_test_results_dir, '_hudson')
        if len(os.listdir(output_dir)) == 0:
          filename = os.path.join(output_dir, "dummy.xml")
          print >> sys.stderr, "No output from cleanunit, writing %s"% filename
          with open(filename, 'w') as f:
            f.write(dummy_test_results_simple)

        self.post_build(failure, test_failure, workspace)

    def post_build(self, failure, test_failure, workspace):
        # If --email was passed in, do a poor-man's version of Hudson's
        # email reporting.  Useful for on-demand builds.
        if self.email and (failure or test_failure):
            subject = None
            if failure:
                subject = '%s - Build # %s - Failure!'%(os.environ['JOB_NAME'], os.environ['BUILD_NUMBER'])
            elif test_failure:
                subject = '%s - Build # %s - Unstable!'%(os.environ['JOB_NAME'], os.environ['BUILD_NUMBER'])
            else:
                subject = '%s - Build # %s - Success!'%(os.environ['JOB_NAME'], os.environ['BUILD_NUMBER'])
            body = 'Check console output at %s to view the results.\n\n'%os.path.join(os.environ['HUDSON_URL'],'job',os.environ['JOB_NAME'],os.environ['BUILD_NUMBER'])
            # Try to make the stack test build results stand out
            # This is harder to do when the user can request multiple
            # stacks
#            if self.stack_data:
#                stack_names = []
#                stack_sources = []
#                for stack in self.stack_data:
#                stack_names.append(stack['name'])
#                if stack['uri']:
#                    stack_sources.append(stack['uri'])
#                else:
#                    stack_sources.append(stack['repo'] + '/trunk')
#                subject = '[%s] %s'%(','.join(stack_names), subject)
#                body += 'Build report on stack(s): \n'
#                for i in range(0,len(stack_names)):
#                    body += '    %s : %s\n'%(stack_names[i], stack_sources[i])
#            body += '\nThe build was done against the %s distribution\n\n'%(self.distro)
            build_failures = os.path.join(os.environ['WORKSPACE'], 'build_output', 'buildfailures.txt')
            build_failures_context = os.path.join(os.environ['WORKSPACE'], 'build_output', 'buildfailures-with-context.txt')
            test_failures = os.path.join(os.environ['WORKSPACE'], 'test_output', 'testfailures.txt')
            if os.path.exists(build_failures):
                body += open(build_failures).read()
                body += '\n\n'
            if os.path.exists(test_failures):
                body += open(test_failures).read()
                body += '\n\n'
            if os.path.exists(build_failures_context):
                body += open(build_failures_context).read()
                body += '\n\n'
            text = subject + '\n\n' + body

            send_mail(EMAIL_FROM_ADDR, self.email, subject, text)

        if failure:
            sys.exit(1)

if __name__ == '__main__':            
    hh = HudsonHelper(sys.argv)
    hh.main()

