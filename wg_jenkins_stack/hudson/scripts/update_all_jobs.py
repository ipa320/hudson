#!/usr/bin/python

import roslib; roslib.load_manifest("job_generation")
from job_generation.jobs_common import *
import hudson
from ast import literal_eval #to convert string to dict

def main():
    try:
        info = get_auth_keys('jenkins', HOME_FOLDER)
        hudson_instance = hudson.Hudson(SERVER, info.group(1), info.group(2))
        
        all_jobs = hudson_instance.get_jobs()
        for job in all_jobs:
            # get repo, distro, rosrelease, arch and githubusername from jobname
            # get job configurations by job_name from jenkins
            # get email from database
            # setup config.xml for reconfiguration

            # hudson_instance.reconfig_job(job['name'], config)
        
    # catch all exceptions
    except Exception, e:
        print 'ERROR: Failed to communicate with Hudson server. Try again later.'
        raise
        

if __name__ == '__main__':
    main()
