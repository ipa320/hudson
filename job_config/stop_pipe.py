#!/usr/bin/python

import sys
import re
import httplib, urllib
import base64
import socket


def main():
    # read parameters
    release = sys.argv[1]
    githubuser = sys.argv[2] 
    init_stack = sys.argv[3]
    
    # get username and password for authentication in jenkins
    gitconfig = open("/home/jenkins/.gitconfig", "r") 
    gitconfig = gitconfig.read()
    # extract necessary data
    regex = ".*\[jenkins]\s*user\s*=\s*([^\s]*)\s*password\s*=\s*([^\s]*).*"
    gitinfo = re.match(regex, gitconfig, re.DOTALL)

    ARCHITECTURE = ['i386', 'amd64']
    UBUNTUDISTRO = ['natty', 'maverick', 'lucid']
    STACKLIST = ['cob_extern', 'cob_common', 'cob_driver', 'cob_simulation', 'cob_apps', 'cob3_intern']
    
    # generate possible job_name
    STACKLIST = STACKLIST[STACKLIST.index(init_stack):]
    for stack in STACKLIST:
        for distro in UBUNTUDISTRO:
                    
            if not stack == init_stack:
                job_name = release + "__" + githubuser + "__" + repo + "__pipe"
                send_stop(job_name, gitinfo.group(1), gitinfo.group(2))
            
            for arch in ARCHITECTURE:
                job_name = release + "__" + githubuser + "__" + repo + "__" + distro + "__" + arch
                send_stop(job_name, gitinfo.group(1), gitinfo.group(2))

        
def send_stop(job_name, username, password):
    # send 'stop' post to jenkins API
    path = '/job/' + job_name + '/lastBuild/stop'

    base64string = base64.encodestring('%s:%s' % (username, password)).strip()
    headers = {"Content-Type": "text/xml", "charset": "UTF-8", "Authorization": "Basic %s" % base64string }
    conn = httplib.HTTPConnection("cob-kitchen-server", 8080)
    conn.request('POST', path, None, headers)
    conn.getresponse()
    conn.close

main()
