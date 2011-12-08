#!/usr/bin/python

import roslib; roslib.load_manifest("job_generation")
from job_generation.jobs_common import *
import hudson

def main():
    try:
        # create hudson instance
        if len(args) == 2:
            hudson_instance = hudson.Hudson(SERVER, args[0], args[1])
        else:
            info = get_auth_keys('jenkins', HOME_FOLDER)
            hudson_instance = hudson.Hudson(SERVER, info.group(1), info.group(2))
        
        broken_jobs = hudson_instance.get_broken_jobs()
        if broken_jobs == []:
            print 'Congratulations, there are no broken jobs'
        else:
            print 'Restarting broken jobs:'
            for broken_job in broken_jobs:
                hudson_instance.build_job(broken_job)
                print '- %s'%str(broken_job)

    # catch all exceptions
    except Exception, e:
        print 'ERROR: Failed to communicate with Hudson server. Try again later.'

if __name__ == '__main__':
    main()
