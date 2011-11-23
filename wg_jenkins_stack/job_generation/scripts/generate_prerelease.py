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
with open("all_config.xml", "r") as f3:
    HUDSON_ALL_CONFIG = f3.read()


def prerelease_job_name(jobtype, rosdistro, stack_list, githubuser, ubuntu, arch):
    return get_job_name(jobtype, rosdistro, githubuser, ubuntu, arch, stack_name='_'.join(stack_list))


def replace_param(hudson_config, rosdistro, githubuser, job_type, arch="", ubuntudistro="", stack_list=[], email="", repeat=0, source_only="", post_jobs=[], not_forked=False):

    hudson_config = hudson_config.replace('BOOTSTRAP_SCRIPT', BOOTSTRAP_SCRIPT)
    hudson_config = hudson_config.replace('SHUTDOWN_SCRIPT', SHUTDOWN_SCRIPT)
    hudson_config = hudson_config.replace('UBUNTUDISTRO', ubuntudistro)
    hudson_config = hudson_config.replace('ARCH', arch)
    hudson_config = hudson_config.replace('ROSDISTRO', rosdistro)
    hudson_config = hudson_config.replace('STACKNAME', '---'.join(stack_list))
    hudson_config = hudson_config.replace('STACKARGS', ' '.join(['--stack %s'%s for s in stack_list]))
    hudson_config = hudson_config.replace('EMAIL_TRIGGERS', get_email_triggers(['Failure', 'StillFailing'], False)) #'Unstable', 'StillUnstable', 'Fixed'
    hudson_config = hudson_config.replace('ADMIN_EMAIL', ADMIN_EMAIL)
    hudson_config = hudson_config.replace('EMAIL', email)
    hudson_config = hudson_config.replace('GITHUBUSER', githubuser)
    hudson_config = hudson_config.replace('REPEAT', str(repeat))
    hudson_config = hudson_config.replace('LABEL', job_type)
    hudson_config = hudson_config.replace('RAMDISK', '--ramdisk --ramdisk-size 20000M')

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


def create_prerelease_configs(rosdistro, stack_list, githubuser, email, repeat, source_only, hudson_obj, arches=None, ubuntudistros=None, not_forked=False, delete=False):
    stack_list.sort()

    if not arches:
        arches = ARCHES
    if not ubuntudistros:
        ubuntudistros = UBUNTU_DISTRO_MAP

    configs = {}
    post_jobs = []
    
    # create pipe_job
    name_pipe = get_job_name(rosdistro, githubuser, stack_name=stack_list, jobtype="pipe")
    configs[name_pipe] = replace_param(HUDSON_PIPE_CONFIG, rosdistro, githubuser, "pipe", stack_list=stack_list, post_jobs=[get_job_name(rosdistro, githubuser, stack_name=stack_list, ubuntu=PRIO_UBUNTUDISTRO, arch=PRIO_ARCH)], not_forked=not_forked)

    # create hudson config files for each ubuntu distro
    for ubuntudistro in ubuntudistros:
        for arch in arches:
            if ubuntudistro != PRIO_UBUNTUDISTRO or arch != PRIO_ARCH: # if job is not prio_job
                name = get_job_name(rosdistro, githubuser, stack_list, ubuntudistro, arch)
                post_jobs.append(name)
                configs[name] = replace_param(HUDSON_CONFIG, rosdistro, githubuser, "build", arch, ubuntudistro, stack_list, email, repeat, source_only)

    # create prio_job
    name = get_job_name(rosdistro, githubuser, stack_list, PRIO_UBUNTUDISTRO, PRIO_ARCH)
    configs[name] = replace_param(HUDSON_CONFIG, rosdistro, githubuser, "build_prio", PRIO_ARCH, PRIO_UBUNTUDISTRO, stack_list, email, repeat, source_only, post_jobs)

    # create 'all' job
    if not delete: # avoid 'all' job to be deleted
      name = get_job_name(rosdistro, githubuser, jobtype="all")
      pipe_job_names = hudson_obj.get_pipe_jobs(rosdistro, githubuser)
      if not name_pipe in pipe_job_names:
          pipe_job_names.append(name_pipe)
      configs[name] = replace_param(HUDSON_ALL_CONFIG, rosdistro, githubuser, "all", post_jobs=pipe_job_names)

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
            info = get_auth_keys('jenkins', HOME_FOLDER)
            hudson_instance = hudson.Hudson(SERVER, info.group(1), info.group(2))

        prerelease_configs = create_prerelease_configs(options.rosdistro, options.stack, options.githubuser, options.email, options.repeat, options.source_only, hudson_instance, options.arch, options.ubuntu, options.not_forked, options.delete)

        # send prerelease tests to Hudson
        print 'Creating pre-release Hudson jobs:<br>'
        schedule_jobs(prerelease_configs,wait=True, start=False, hudson_obj=hudson_instance, delete=options.delete)
        if options.delete:
            print '<br>Jobs have been deleted. You can now start new jobs<br>'
        else:
            print '<br><b>%s</b> will receive %d emails on <b>%s</b>, one for each job<br>'%(options.githubuser, len(prerelease_configs), options.email)
            print 'You can follow the progress of these jobs on the <b> <a href="%s"> Jenkins Server</a> </b> <br>'%(SERVER)

    # catch all exceptions
    except Exception, e:
        print 'ERROR: Failed to communicate with Hudson server. Try again later.'

if __name__ == '__main__':
    main()
