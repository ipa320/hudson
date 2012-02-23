#!/usr/bin/python

import roslib; roslib.load_manifest("job_generation")
from job_generation.jobs_common import *
import hudson
from ast import literal_eval #to convert string to dict
import os

def main():
    try:
        info = get_auth_keys('jenkins', HOME_FOLDER)
        hudson_instance = hudson.Hudson(SERVER, info.group(1), info.group(2))
        
        all_jobs = hudson_instance.get_jobs()
        for job in all_jobs:
            repo_list = []
            post_jobs = []
            if "__all" in job['name'] or "a_restart_" in job['name'] or "a_update_" in job['name']:
                # those jobs should not be updated with this script
                continue 
            elif "__pipe" in job['name']:
                #====================================================
                # get rosrelease, githubusername and repo from jobname
                param_list = job['name'].split('__')
                rosrelease = param_list[0]
                githubuser = param_list[1]
                repo_list.append(param_list[2]) # list 

                # POSTJOBS
                job_info = hudson_instance.get_job_info(job['name'])

                for post_job in job_info['downstreamProjects']:
                    post_jobs.append(post_job['name'])

                # setup config.xml for reconfiguration
                config_file = os.path.join(HOME_FOLDER, "git/hudson/wg_jenkins_stack/job_generation/scripts/pipe_config.xml")
                with open(config_file, "r") as f:
                    HUDSON_PIPE_CONFIG = f.read()
                config = replace_param(HUDSON_PIPE_CONFIG, rosrelease, githubuser, "pipe", stack_list=repo_list, post_jobs=post_jobs)
                
                

            else: # all build jobs
                #====================================================
                # get rosrelease, githubusername, repo, distro and arch from jobname
                param_list = job['name'].split('__')
                rosrelease = param_list[0]
                githubuser = param_list[1]
                repo_list.append(param_list[2]) # list 
                ubuntudistro = param_list[3]
                arch = param_list[4]
                # LABEL
                if ubuntudistro == PRIO_UBUNTUDISTRO and arch == PRIO_ARCH:
                    label = "build_prio"
                else:
                    label = "build"


                #====================================================
                # get job configurations by job_name from jenkins
                # POSTJOBS
                job_info = hudson_instance.get_job_info(job['name'])

                for post_job in job_info['downstreamProjects']:
                    post_jobs.append(post_job['name'])

                #====================================================
                # get job configurations from job_common.py
                # BOOTSTRAP_SCRIPT, SHUTDOWN_SCRIPT, ADMIN_EMAIL

                # get email from database
                user_database = os.path.join(HOME_FOLDER, "jenkins_users")
                with open(user_database, "r") as f:
                    user_dict = literal_eval(f.read())
                if githubuser not in user_dict:
                    print 'ERROR: User not in database!'
                    raise
                else:
                    email = user_dict.get(githubuser)

                #====================================================
                # setup config.xml for reconfiguration
                config_file = os.path.join(HOME_FOLDER, "git/hudson/wg_jenkins_stack/job_generation/scripts/build_config.xml")
                with open(config_file, "r") as f:
                    HUDSON_CONFIG = f.read()
                config = replace_param(HUDSON_CONFIG, rosrelease, githubuser, label, arch, ubuntudistro, repo_list, email, post_jobs=post_jobs)


            #====================================================
            # reconfigure job 
            try:
                hudson_instance.reconfig_job(job['name'], config)
            except:
                print 'ERROR: Could not reconfigure job!'
                raise
        
            print 'reconfigured: %s'%job['name']

    # catch all exceptions
    except Exception, e:
        print 'ERROR: Failed to communicate with Hudson server. Try again later.'
        raise
        

if __name__ == '__main__':
    main()
