#!/usr/bin/python

import roslib; roslib.load_manifest("job_generation")
from roslib import stack_manifest
import optparse
import hudson
import urllib
import time
import re
import os
import rosdistro
import StringIO
import pycurl
import subprocess
import socket
import yaml



BOOTSTRAP_SCRIPT = """
sudo chown -R jenkins:jenkins $WORKSPACE
rm -rf $WORKSPACE/test_results
rm -rf $WORKSPACE/test_output
rm -rf $WORKSPACE/hudson


#check whether someone else is logged in
echo "Checking if someone is logged in"
THREADS=""

USERCOUNT=$(who -q | grep 'users')
if [ ${USERCOUNT: -1} != 0 ]
  then
    COUNT=$(cat /proc/cpuinfo | grep 'processor' | wc -l)
    COUNT=$(echo "$COUNT/2" | bc)
    THREADS="--threads="$COUNT
    echo "Because someone else is logged in, only half of the cores will be used"
fi 


cat &gt; $WORKSPACE/script.sh &lt;&lt;DELIM
#!/usr/bin/env bash
set -o errexit
echo "_________________________________BEGIN SCRIPT______________________________________"
echo ""
echo "***********************************************************************************"
echo "INSTALLING ros distribution, bzr and python-pycurl"
sudo apt-get install bzr --yes
sudo apt-get install ros-ROSDISTRO-ros --yes
sudo apt-get install python-pycurl
echo "***********************************************************************************"
echo ""
source /opt/ros/ROSDISTRO/setup.sh

export INSTALL_DIR=/tmp/install_dir
export WORKSPACE=/tmp/ros
export ROS_TEST_RESULTS_DIR=/tmp/ros/test_results
export JOB_NAME=$JOB_NAME
export BUILD_NUMBER=$BUILD_NUMBER
export HUDSON_URL=$HUDSON_URL
export ROS_PACKAGE_PATH=\$INSTALL_DIR/ros_release:/opt/ros/ROSDISTRO/stacks

mkdir -p \$INSTALL_DIR
cd \$INSTALL_DIR

cp /tmp/workspace/.gitconfig ~/.gitconfig
cp -r /tmp/workspace/.ssh ~/.ssh
sudo chmod 600 ~/.ssh/id_rsa.pub ~/.ssh/id_rsa

sudo mkdir ros_release
sudo mv -f /tmp/workspace/hudson/wg_jenkins_stack/* ./ros_release
#ls -la
#ls -la ros_release/
#cp ros_release/hudson/src/hudson_helper_fhg.py .
#sudo chmod +x  hudson_helper_fhg.py
""" 


SHUTDOWN_SCRIPT = """
echo "_________________________________END SCRIPT_______________________________________"
DELIM

set -o errexit

scp jenkins@cob-kitchen-server:/home/jenkins/jenkins-config/.gitconfig $WORKSPACE/.gitconfig
scp -r jenkins@cob-kitchen-server:/home/jenkins/jenkins-config/.ssh $WORKSPACE/.ssh

git clone git://github.com/ipa320/hudson.git $WORKSPACE/hudson

cd $WORKSPACE &amp;&amp; $WORKSPACE/hudson/wg_jenkins_stack/hudson/scripts/devel_run_chroot.py --chroot-dir $HOME/chroot --distro=UBUNTUDISTRO --arch=ARCH --debug-chroot  --hdd-scratch=/home/rosbuild/install_dir --script=$WORKSPACE/script.sh --repo-url http://cob-kitchen-server:3142/de.archive.ubuntu.com/ubuntu RAMDISK
"""

# the supported Ubuntu distro's for each ros distro
ARCHES = ['amd64', 'i386']

# ubuntu distro mapping to ros distro
UBUNTU_DISTRO_MAP = ['lucid', 'maverick', 'natty'] #, 'oneiric']


# Path to hudson server
SERVER = 'http://%s:8080'%socket.gethostname()

# Local HOME path
if socket.gethostname() == "cob-kitchen-server":
    HOME_FOLDER = '/home/jenkins'
else:
    HOME_FOLDER = '/home-local/jenkins'

# list of public and private IPA Fraunhofer stacks
FHG_STACKS_PUBLIC = ['cob_extern', 'cob_common', 'cob_calibration', 'cob_driver', 'cob_driver_sandbox', 'cob_robots', 'cob_environments', 'cob_simulation', 'cob_apps', 'cob_manipulation', 'cob_manipulation_sandbox', 'cob_object_manipulation', 'cob_navigation', 'cob_environment_perception', 'cob_people_perception', 'cob_object_perception', 'cob_scenarios', 'cob_web', 'cob_command_tools', 'schunk_modular_robotics', 'schunk_robots']
FHG_STACKS_PRIVATE = ['cob_manipulation_intern', 'cob_navigation_intern', 'cob_environment_perception_intern', 'cob_object_perception_intern', 'cob_scenarios_intern', 'cob_sandbox_intern', 'interaid', 'srs', 'r3cop', 'autopnp']

PRIO_ARCH = "i386"
PRIO_UBUNTUDISTRO = "natty"

#email addresses of administrators
ADMIN_EMAIL = "fmw@ipa.fhg.de"

EMAIL_TRIGGER="""
        <hudson.plugins.emailext.plugins.trigger.WHENTrigger> 
          <email> 
            <recipientList></recipientList> 
            <subject>$PROJECT_DEFAULT_SUBJECT</subject> 
            <body>$PROJECT_DEFAULT_CONTENT</body> 
            <sendToDevelopers>SEND_DEVEL</sendToDevelopers> 
            <sendToRecipientList>true</sendToRecipientList> 
            <contentTypeHTML>false</contentTypeHTML> 
            <script>true</script> 
          </email> 
        </hudson.plugins.emailext.plugins.trigger.WHENTrigger> 
"""


def stack_to_rosinstall(stack_obj, branch):
    try:
        return yaml.dump(rosdistro.stack_to_rosinstall(stack_obj, branch, anonymous=True))
    except rosdistro.DistroException, ex:
        print str(ex)
        return ''


def stack_to_deb(stack, rosdistro):
    return '-'.join(['ros', rosdistro, str(stack).replace('_','-')])

def stacks_to_debs(stack_list, rosdistro):
    if not stack_list or len(stack_list) == 0:
        return ''
    return ' '.join([stack_to_deb(s, rosdistro) for s in stack_list])


def get_depends_one(stack_name, githubuser, spaces=""):
    # get stack.xml from github
    stack_xml = get_stack_xml(stack_name, githubuser)
    # convert to list
    depends_one = [str(d) for d in stack_manifest.parse(stack_xml).depends]
    print spaces, 'Dependencies of stack %s:'%stack_name
    print spaces, str(depends_one)
    return depends_one


def get_depends_all(stack_name, depends_all, githubuser, start_depth):
    depends_all_list = []
    # convert depends_all entries to list
    [[depends_all_list.append(value) for value in valuelist] for valuelist in depends_all.itervalues()]
    if not stack_name in depends_all_list: # new stack and not in depends_all
        print " "*2*start_depth, start_depth, "+ Included %s to dependencies"%stack_name
        # append stack to the right list in depends_all
        depends_all[get_stack_membership(stack_name)].append(stack_name)
        # find and append all IPA dependencies
        if stack_name in FHG_STACKS_PRIVATE or stack_name in FHG_STACKS_PUBLIC:
            for d in get_depends_one(stack_name, githubuser, " "*2*start_depth):
                get_depends_all(d, depends_all, githubuser, start_depth+1)
    else:
        print " "*2*start_depth, start_depth, "- %s already included"%stack_name


def get_stack_membership(stack_name):
    if stack_name in FHG_STACKS_PUBLIC:
        return "public"
    elif stack_name in FHG_STACKS_PRIVATE:
        return "private"
    else:
        return "other"


def stack_forked(githubuser, stack_name, appendix="/blob/master/Makefile"):
    git_auth = get_auth_keys('github', "/tmp/workspace")
    post = {'login' : git_auth.group(1), 'token' : git_auth.group(2)}
    fields = urllib.urlencode(post)
    path = "https://github.com/" + githubuser + "/" + stack_name + appendix
    #print path
    #print fields
    file1 = StringIO.StringIO()
    c = pycurl.Curl()
    c.setopt(pycurl.URL, path)
    c.setopt(pycurl.POSTFIELDS, fields)
    c.setopt(pycurl.WRITEFUNCTION, file1.write) # to avoid to show the called page
    c.perform()
    c.close

    if c.getinfo(pycurl.HTTP_CODE) == 200:
        #print "Stack found"
        return True
    else:
        print "PATH: ", path
        print "ERRORCODE: ", c.getinfo(pycurl.HTTP_CODE)
        return False


def stack_released(stack_name, rosdistro, env):
    print 'Checking if stack is released'
    pkg_name = stack_to_deb(stack_name, rosdistro)
    err_msg = call('sudo apt-get -s install %s'%pkg_name, env, ignore_fail=True, quiet=True)
    #print "ERROR MESSAGE: ", err_msg
    if "E: Unable to locate package %s"%pkg_name in err_msg:
        print '%s is not released'%stack_name
        return False
    else:
        print '%s is released'%stack_name
        return True

def stack_origin(rosdistro_obj, rosinstall, stack_name, githubuser, overlay_dir, env):
    # check if stack is private, public or other / forked or not / released or not
    # gives back rosinstall entry or clones stack in case it is private
    
    if stack_name in FHG_STACKS_PRIVATE:    # stack is private ipa stack
        print "Stack %s is a private ipa stack" %(stack_name)
        if not stack_forked(githubuser, stack_name):    # check if stack is forked for user or not
            print "  Stack %s is not forked for user %s" %(stack_name, githubuser)
            githubuser = 'ipa320'
            if stack_released(stack_name, rosdistro_obj.release_name, env):  # stack is released
                print "    Using released version"
                call('sudo apt-get install %s --yes'%(stack_to_deb(stack_name, rosdistro_obj.release_name)), env, 'Install released version')
                return ''
            print "    Using 'ipa320' stack instead\n"    # stack is not released, using 'ipa320' fork
        call('git clone git@github.com:%s/%s.git %s/%s'%(githubuser, stack_name, overlay_dir, stack_name), env, 'Clone private stack [%s] to test'%(stack_name))
        return ''
        
    elif stack_name in FHG_STACKS_PUBLIC:   # stack is public ipa stack
        print "Stack %s is a public ipa stack" %(stack_name)
        if not stack_forked(githubuser, stack_name):    # check if stack is forked for user or not
            print "  Stack %s is not forked for user %s" %(stack_name, githubuser)
            githubuser = 'ipa320'
            if stack_released(stack_name, rosdistro_obj.release_name, env):  # stack is released
                print "    Using released version"
                call('sudo apt-get install %s --yes'%(stack_to_deb(stack_name, rosdistro_obj.release_name)), env, 'Install released version')
                return ''
                #return '- git: {local-name: %s, uri: "git://github.com/ipa320/%s.git", branch-name: %s}\n'%(stack_name, stack_name, rosdistro_obj.release_name)
                #return stack_to_rosinstall(rosdistro_obj.stacks[stack_name], 'release_%s'%rosdistro_obj.release_name)
            print "    Using 'ipa320' stack instead\n"    # stack is not released, using 'ipa320' fork
        return  '- git: {local-name: %s, uri: "git://github.com/%s/%s.git", branch-name: master}\n'%(stack_name, githubuser, stack_name)
        
    elif stack_name in rosdistro_obj.stacks:    # stack is no ipa stack
        print "Stack %s is not a ipa stack, using released version" %(stack_name)
        return stack_to_rosinstall(rosdistro_obj.stacks[stack_name], 'devel')
        
    else:   # stack is no known stack
        raise Exception("ERROR: Stack %s not found! This should never happen!"%(stack_name))
        

def get_stack_xml(stack_name, githubuser, appendix="/master/stack.xml"):
    if not stack_forked(githubuser, stack_name):
        githubuser = "ipa320"

    try:
        git_auth = get_auth_keys('github', '/tmp/workspace')
        post = {'login' : git_auth.group(1), 'token' : git_auth.group(2)}
        fields = urllib.urlencode(post)
        path = "https://raw.github.com/" + githubuser + "/" + stack_name + appendix
        tmpfile = StringIO.StringIO()

        c = pycurl.Curl()
        c.setopt(pycurl.URL, path)
        c.setopt(pycurl.POSTFIELDS, fields)
        c.setopt(pycurl.WRITEFUNCTION, tmpfile.write)
        c.perform()
        stack_xml = tmpfile.getvalue()

        c.close
    except :
        #TODO
        pass
    return stack_xml
    
        
def get_auth_keys(server, location):
    # get password/token from .gitconfig file
    with open(location + "/.gitconfig", "r") as f:
        gitconfig = f.read()
        # extract necessary data
        if server == "github":
            regex = ".*\[" + server + "]\s*user\s*=\s*([^\s]*)\s*token\s*=\s*([^\s]*).*"
            
        elif server == "jenkins":
            regex = ".*\[" + server + "]\s*user\s*=\s*([^\s]*)\s*password\s*=\s*([^\s]*).*"
        else:
            print "ERROR: invalid server"
            # TODO error raise
            
        auth_keys = re.match(regex, gitconfig, re.DOTALL)
        
    return auth_keys


def get_environment():
    my_env = os.environ
    my_env['WORKSPACE'] = os.getenv('WORKSPACE', '')
    my_env['INSTALL_DIR'] = os.getenv('INSTALL_DIR', '')
    #my_env['HOME'] = os.getenv('HOME', '')
    my_env['HOME'] = os.path.expanduser('~')
    my_env['JOB_NAME'] = os.getenv('JOB_NAME', '')
    my_env['BUILD_NUMBER'] = os.getenv('BUILD_NUMBER', '')
    my_env['ROS_TEST_RESULTS_DIR'] = os.getenv('ROS_TEST_RESULTS_DIR', my_env['WORKSPACE']+'/test_results')
    my_env['PWD'] = os.getenv('WORKSPACE', '')
    return my_env


def get_options(required, optional):
    parser = optparse.OptionParser()
    ops = required + optional
    if 'os' in ops:
        parser.add_option('--os', dest = 'os', default='ubuntu', action='store',
                          help='OS name')
    if 'rosdistro' in ops:
        parser.add_option('--rosdistro', dest = 'rosdistro', default=None, action='store',
                          help='Ros distro name')
    if 'stack' in ops:
        parser.add_option('--stack', dest = 'stack', default=None, action='append',
                          help='Stack name')
    if 'githubuser' in ops:
        parser.add_option('--githubuser', dest = 'githubuser', default=None, action="store",
                          help='Login name of github account')
    if 'email' in ops:
        parser.add_option('--email', dest = 'email', default=None, action='store',
                          help='Email address to send results to')
    if 'arch' in ops:
        parser.add_option('--arch', dest = 'arch', default=None, action='append',
                          help='Architecture to test')
    if 'ubuntu' in ops:
        parser.add_option('--ubuntu', dest = 'ubuntu', default=None, action='append',
                          help='Ubuntu distribution to test')
    if 'repeat' in ops:
        parser.add_option('--repeat', dest = 'repeat', default=0, action='store',
                          help='How many times to repeat the test')
    if 'source-only' in ops:
        parser.add_option('--source-only', dest = 'source_only', default=False, action='store_true',
                          help="Build everything from source, don't use Debian packages")
    if 'delete' in ops:
        parser.add_option('--delete', dest = 'delete', default=False, action='store_true',
                          help='Delete jobs from Hudson')    
    if 'wait' in ops:
        parser.add_option('--wait', dest = 'wait', default=False, action='store_true',
                          help='Wait for running jobs to finish to reconfigure them')    
    if 'rosinstall' in ops:
        parser.add_option('--rosinstall', dest = 'rosinstall', default=None, action='store',
                          help="Specify the rosinstall file that refers to unreleased code.")
    if 'overlay' in ops:
        parser.add_option('--overlay', dest = 'overlay', default=None, action='store',
                          help='Create overlay file')
    if 'variant' in ops:
        parser.add_option('--variant', dest = 'variant', default=None, action='store',
                          help="Specify the variant to create a rosinstall for")
    if 'database' in ops:
        parser.add_option('--database', dest = 'database', default=None, action='store',
                          help="Specify database file")
    if 'not-forked' in ops:
        parser.add_option('--not-forked', dest = 'not_forked', default=False, action='store_true',
                          help="Stack is not forked for given githubuser")
    if 'threads' in ops:
        parser.add_option('--threads', dest = 'threads', default=0, action='store',
                          help="Build up to N packages in parallel")

    (options, args) = parser.parse_args()
    

    # make repeat an int
    if 'repeat' in ops:
        options.repeat = int(options.repeat)
    
    # make threads an int
    if 'threads' in ops:
        options.threads = int(options.threads)

    # check if required arguments are there
    for r in required:
        if not eval('options.%s'%r):
            print 'You need to specify "--%s"'%r
            return (None, args)

    # postprocessing
    if 'email' in ops and options.email and not '@' in options.email:
        print 'You provided an invalid email address'
        return (None, args)


    # check if rosdistro exists
#    if 'rosdistro' in ops and (not options.rosdistro or not options.rosdistro in UBUNTU_DISTRO_MAP.keys()):
#        print 'You provided an invalid "--rosdistro %s" argument. Options are %s'%(options.rosdistro, UBUNTU_DISTRO_MAP.keys())
#        return (None, args)

    # check if stacks exist
    #if 'stack' in ops and options.stack:
        #distro_obj = rosdistro.Distro(get_rosdistro_file(options.rosdistro))
        #for s in options.stack:
            #if not s in distro_obj.stacks:
                #print 'Stack "%s" does not exist in the %s disro file.'%(s, options.rosdistro)
                #print 'You need to add this stack to the rosdistro file'
                #return (None, args)

    # check if variant exists
    if 'variant' in ops and options.variant:
        distro_obj = rosdistro.Distro(get_rosdistro_file(options.rosdistro))
        if not options.variant in distro_obj.variants:
                print 'Variant "%s" does not exist in the %s disro file.'%(options.variant, options.rosdistro)
                return (None, args)

    return (options, args)


def schedule_jobs(jobs, wait=False, delete=False, start=False, hudson_obj=None):
    # create hudson instance
    if not hudson_obj:
        info = get_auth_keys('jenkins', HOME_FOLDER) ################
        hudson_obj = hudson.Hudson(SERVER, info.group(1), info.group(2))

    finished = False
    while not finished:
        jobs_todo = {}
        for job_name in jobs:
            if 'pipe' in job_name:
                start=True
            else:
                start=False
            exists = hudson_obj.job_exists(job_name)
            
            # cancel all jobs of stack in the build queue 
            if 'pipe' in job_name:
                build_queue = hudson_obj.get_queue_info()
                job_name_stem = job_name.replace('pipe', '')
                if build_queue != []:
                    for pending_job in build_queue:
                        if job_name_stem in pending_job['task']['name']:
                            hudson_obj.cancel_pending_job(pending_job['id'])
                            print "Canceling pending job %s from build queue <br>"%(pending_job['task']['name'])

            # job is already running
            if exists and hudson_obj.job_is_running(job_name):           
                hudson_obj.stop_running_job(job_name)
                jobs_todo[job_name] = jobs[job_name]
                print "Job %s is running! Stopping job to reconfigure <br>"%job_name

            # delete pending job from queue
            elif exists and hudson_obj.job_in_queue(job_name):
                hudson_obj.cancel_pending_job(job_name)
                jobs_todo[job_name] = jobs[job_name]
                print "Job %s is in build queue! Cancel pending build <br>"%job_name

            # delete old job
            elif delete:
                if exists:
                    hudson_obj.delete_job(job_name)
                    print " - Deleting job %s <br>"%job_name

            # reconfigure job
            elif exists:
                hudson_obj.reconfig_job(job_name, jobs[job_name])
                if start:
                    hudson_obj.build_job(job_name)
                print " - %s <br>"%job_name

            # create job
            elif not exists:
                hudson_obj.create_job(job_name, jobs[job_name])
                if start:
                    hudson_obj.build_job(job_name)
                print " - %s <br>"%job_name

        if wait and len(jobs_todo) > 0:
            jobs = jobs_todo
            jobs_todo = {}
            time.sleep(1.0)
        else:
            finished = True


def get_rosdistro_file(rosdistro):
    return 'https://code.ros.org/svn/release/trunk/distros/%s.rosdistro'%rosdistro


def get_email_triggers(when, send_devel=True):
    triggers = ''
    for w in when:
        trigger = EMAIL_TRIGGER
        trigger = trigger.replace('WHEN', w)
        if send_devel:
            trigger = trigger.replace('SEND_DEVEL', 'true')
        else:
            trigger = trigger.replace('SEND_DEVEL', 'false')
        triggers += trigger
    return triggers


def get_job_name(rosdistro, githubuser, stack_name="", ubuntu="", arch="", jobtype=""):
    if len(stack_name) > 50:
        stack_name = stack_name[0:46]+'_...'
    stack_name = "_".join(stack_name)
    if jobtype == "pipe":
        return "__".join([rosdistro, githubuser, stack_name, jobtype])
    elif jobtype == "all":
        return "__".join([rosdistro, githubuser, jobtype])
    else:
        return "__".join([rosdistro, githubuser, stack_name, ubuntu, arch])


def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)

def write_file(filename, msg):
    ensure_dir(filename)
    with open(filename, 'w') as f:
        f.write(msg)

def generate_email(message, env):
    print message
    write_file(env['WORKSPACE']+'/build_output/buildfailures.txt', message)
    write_file(env['WORKSPACE']+'/test_output/testfailures.txt', '')
    write_file(env['WORKSPACE']+'/build_output/buildfailures-with-context.txt', '')


def call(command, env=None, message='', ignore_fail=False, quiet=False):
    res = ''
    err = ''
    try:
        print message+'\nExecuting command "%s"'%command
        helper = subprocess.Popen(command.split(' '), stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        res, err = helper.communicate()
        if not quiet:
            print str(res)
            print str(err)
        if helper.returncode != 0:
            raise Exception
        return res
    except Exception:
        if not ignore_fail:
            message += "\n=========================================\n"
            message += "Failed to execute '%s'"%command
            message += "\n=========================================\n"
            message += str(res)
            message += "\n=========================================\n"
            message += str(err)
            message += "\n=========================================\n"
            if env:
                message += "ROS_PACKAGE_PATH = %s\n"%env['ROS_PACKAGE_PATH']
                message += "ROS_ROOT = %s\n"%env['ROS_ROOT']
                message += "PYTHONPATH = %s\n"%env['PYTHONPATH']
                message += "\n=========================================\n"
                generate_email(message, env)
            raise Exception
        else:
            return str(err)
