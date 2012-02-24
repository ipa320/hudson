#!/usr/bin/python

import roslib; roslib.load_manifest("job_generation")
from job_generation.jobs_common import *
import hudson
import re
from datetime import date, timedelta

def main():
    try:
        old_jobs = [];
        regex = "\d+"
                
        info = get_auth_keys('jenkins', HOME_FOLDER)
        hudson_instance = hudson.Hudson(SERVER, info.group(1), info.group(2))

        today = date.today()
        offset = timedelta(days=60)
        
        all_jobs = hudson_instance.get_jobs()
        for job in all_jobs:
            if not ( 'a_restart' in job['name'] or 'a_update' in job['name'] or '__all' in job['name']):
                if not (hudson_instance.job_in_queue(job['name']) or hudson_instance.job_is_running(job['name'])):
                    date_string = hudson_instance.get_last_build_date(job['name'])
                    if data_string != '0001-01-01_00-00-00':
			    date_list = re.findall(regex, date_string)
			    build_year = int(date_list[0])
			    build_month = int(date_list[1])
			    build_day = int(date_list[2])
			    build_date = date(build_year, build_month, build_day)
			    build_date += offset
			    
			    if build_date <= today:
				old_jobs.append(job['name'])
            
        if old_jobs == []:
            print 'No jobs older than 2 months!'
        else:
            print 'Restarting all jobs older than 2 months:'
            for old_job in old_jobs:
                hudson_instance.build_job(old_job)
                print '- %s'%str(old_job)
            
        
    # catch all exceptions
    except Exception, e:
        print 'ERROR: Failed to communicate with Hudson server. Try again later.'
        raise

if __name__ == '__main__':
    main()
