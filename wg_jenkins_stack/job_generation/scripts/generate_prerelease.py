#!/usr/bin/python

import roslib; roslib.load_manifest("job_generation")
from job_generation.jobs_common import *
import hudson


# template to create pre-release hudson configuration file
#TODO file location
with open("build_config.xml", "r") as f1:
    HUDSON_CONFIG = f1.read()
with open("pipe_config.xml", "r") as f2:
    HUDSON_PIPE_CONFIG = f2.read()


def prerelease_job_name(jobtype, rosdistro, stack_list, githubuser, ubuntu, arch):
    return get_job_name(jobtype, rosdistro, '_'.join(stack_list), githubuser, ubuntu, arch)


def replace_param(hudson_config, rosdistro, githubuser, job_type, arch="", ubuntudistro="", stack_list=[], email="", repeat=0, source_only="", post_jobs=[], not_forked=False):

    hudson_config = hudson_config.replace('BOOTSTRAP_SCRIPT', BOOTSTRAP_SCRIPT)
    hudson_config = hudson_config.replace('SHUTDOWN_SCRIPT', SHUTDOWN_SCRIPT)
    hudson_config = hudson_config.replace('UBUNTUDISTRO', ubuntudistro)
    hudson_config = hudson_config.replace('ARCH', arch)
    hudson_config = hudson_config.replace('ROSDISTRO', rosdistro)
    hudson_config = hudson_config.replace('STACKNAME', '---'.join(stack_list))
    hudson_config = hudson_config.replace('STACKARGS', ' '.join(['--stack %s'%s for s in stack_list]))
    hudson_config = hudson_config.replace('EMAIL_TRIGGERS', get_email_triggers(['Unstable', 'Failure', 'StillFailing', 'Fixed', 'StillUnstable'], False))
    hudson_config = hudson_config.replace('EMAIL', email)
    hudson_config = hudson_config.replace('GITHUBUSER', githubuser)
    hudson_config = hudson_config.replace('REPEAT', str(repeat))
    hudson_config = hudson_config.replace('LABEL', job_type)

    if post_jobs != []:
        hudson_config = hudson_config.replace('POSTJOBS', ', '.join(str(n) for n in post_jobs))
    else:
        hudson_config = hudson_config.replace('POSTJOBS', '')
    
    if source_only:
        hudson_config = hudson_config.replace('SOURCE_ONLY', '--source-only')
    else:
        hudson_config = hudson_config.replace('SOURCE_ONLY', '')
    
    if not_forked:
        hudson_config = hudson_config.replace('STACKOWNER', 'ipa320')
    else:
        hudson_config = hudson_config.replace('STACKOWNER', githubuser)

    return hudson_config


def create_prerelease_configs(rosdistro, stack_list, githubuser, email, repeat, source_only, arches=None, ubuntudistros=None, not_forked=False):
    stack_list.sort()
    
    if not arches:
        arches = ARCHES
    if not ubuntudistros:
        ubuntudistros = UBUNTU_DISTRO_MAP

    prio_arch = "i386"
    prio_ubuntudistro = "natty"
    configs = {}
    post_jobs = []
    
    # create pipe_job
    name = get_job_name(rosdistro, stack_list, githubuser, jobtype="pipe")
    configs[name] = replace_param(HUDSON_PIPE_CONFIG, rosdistro, githubuser, "pipe", stack_list=stack_list, post_jobs=[get_job_name(rosdistro, stack_list, githubuser, ubuntu=prio_ubuntudistro, arch=prio_arch)], not_forked=not_forked)
    
    # create hudson config files for each ubuntu distro
    for ubuntudistro in ubuntudistros:
        for arch in arches:
            if ubuntudistro != prio_ubuntudistro or arch != prio_arch: # if job is not prio_job
                name = get_job_name(rosdistro, stack_list, githubuser, ubuntudistro, arch)
                post_jobs.append(name)
                configs[name] = replace_param(HUDSON_CONFIG, rosdistro, githubuser, "build", arch, ubuntudistro, stack_list, email, repeat, source_only)
    
    # create prio_job
    name = get_job_name(rosdistro, stack_list, githubuser, prio_ubuntudistro, prio_arch)
    configs[name] = replace_param(HUDSON_CONFIG, rosdistro, githubuser, "build_prio", prio_arch, prio_ubuntudistro, stack_list, email, repeat, source_only, post_jobs)

    return configs


def main():
    (options, args) = get_options(['stack', 'rosdistro', 'githubuser', 'email'], ['repeat', 'source-only', 'arch', 'ubuntu', 'delete', 'not-forked'])
    if not options:
        return -1
    
    try:
        # create hudson instance
        if len(args) == 2:
            hudson_instance = hudson.Hudson(SERVER, args[0], args[1])
        else:
            info = get_auth_keys('jenkins', '/home-local/jenkins')
            hudson_instance = hudson.Hudson(SERVER, info.group(1), info.group(2))
        prerelease_configs = create_prerelease_configs(options.rosdistro, options.stack, options.githubuser, options.email, options.repeat, options.source_only, options.arch, options.ubuntu, options.not_forked)
        
        #TODO necessary??? change???
        # check if jobs are not already running
        for job_name in prerelease_configs:
            exists = hudson_instance.job_exists(job_name)
            if exists and hudson_instance.job_is_running(job_name):
                print 'Cannot create job %s because a job with the same name is already running.'%job_name
                print 'Please try again when this job finished running.'
                return 

        # send prerelease tests to Hudson
        print 'Creating pre-release Hudson jobs:<br>'
        schedule_jobs(prerelease_configs, start=False, hudson_obj=hudson_instance, delete=options.delete)
        if options.delete:
            print 'Jobs have been deleted. You can now start new jobs<br>'
        else:
            print '%s will receive %d emails on %s, one for each job<br>'%(options.githubuser, len(prerelease_configs), options.email)
            print 'You can follow the progress of these jobs on <%s><br>'%(SERVER)

    # catch all exceptions
    except Exception, e:
        print 'ERROR: Failed to communicate with Hudson server. Try again later.'

if __name__ == '__main__':
    main()
