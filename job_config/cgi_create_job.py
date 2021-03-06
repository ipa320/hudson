#!/usr/bin/python

import cgi
import cgitb; cgitb.enable()
import httplib, urllib
import sys
import base64
import socket
import StringIO
import pycurl
import re

repositories = []

def main():
    
    global repositories
    rosrelease = []
    
    print "Content-Type: text/html\n\n"     # HTML is following

    form = cgi.FieldStorage() # keys from HTML form

    # check if necessary keys (username & email) are available
    if "username" not in form: # raise error if not
        print "<H1>ERROR<H1>"
        print "Please fill in your Github username and email address."
        print '<p><input type=button value="Back" onClick="history.back()">'
        return

    if "email" not in form and ( form["username"].value != "ipa-fmw" or form["username"].value != "ipa320" ):
        print "<H1>ERROR<H1>"
        print "Please fill in your Github username and email address."
        print '<p><input type=button value="Back" onClick="history.back()">'
        return

    # if available check other parameters
    else:
        print "<p>Creating jobs for:<br>"
        print "<ul><p>Username: ", form["username"].value  
        print "<p>Email: ", form["email"].value, "</ul>"

        # get chosen releases
        releases = form.getlist('release')
        if releases == []:
            print "<H1>ERROR<H1>"
            print "You have to select at least one release! <br>"
            print '<input type=button value="Back" onClick="history.back()">'
            return
        else:
            for release in releases:
                rosrelease = rosrelease[:] + [release]

        # check chosen stacks
        otherstacks = form.getlist('otherstack')
        if form['stacks'].value == 'All':
            repositories = ['cob_apps', 'cob_common', 'cob_driver', 'cob_extern', 'cob_simulation']
        else:
            stacks = form.getlist('stack')
            if stacks == [] and otherstacks == []:
                print "<H1>ERROR<H1>"
                print "You have to select at least one stack! <br>"
                print '<input type=button value="Back" onClick="history.back()">'
                return
            else:
                for stack in stacks:
                    repositories = repositories[:] + [stack]

        if otherstacks != []:
            for stack in otherstacks:
                find_stack(stack)

        # printing planed job creations
        print "<p>Creating jobs to test:<br><ul>"    
        for stack in repositories:
            print "- ", stack, "<br>"
        print "</ul>"
        print "<hr>"

        print spawn_jobs(form["username"].value, form["email"].value, repositories, rosrelease)

    print '<p><input type=button value="Back" onClick="history.back()">'    

def spawn_jobs(githubuser, email, REPOSITORY, ROSRELEASES):
    # function to spawn jobs

    results = """<p>JOB CREATION RESULTS<br>
====================<br>\n"""
    
    # all available options
    ARCHITECTURE = ['i386', 'amd64'] # i686
    UBUNTUDISTRO = ['lucid', 'maverick','natty'] # karmic
    prio_arch = 'i386'
    prio_distro = 'natty'

    for release in ROSRELEASES:
        for repo in REPOSITORY:
            results = results + "<br>"

            # check if stack is forked
            if not stack_forked(githubuser, repo):
                results = results + "<b>" + repo + "</b>" + ": stack is not forked\n"
                continue

            post_jobs = [release + "__" + githubuser + "__" + repo + "__" + prio_distro + "__" + prio_arch]
            results = results + create_job("pipe", githubuser, release, repo, post_jobs=post_jobs)

            post_jobs = []

            for distro in UBUNTUDISTRO:
                for arch in ARCHITECTURE:
                    if (not distro == prio_distro) or (not arch == prio_arch): # if job is not prio_job
                        post_jobs.append(release + "__" + githubuser + "__" + repo + "__" + distro + "__" + arch) # append all missing jobs for repo
                        results = results + create_job("build", githubuser, release, repo, email, distro, arch)
            results = results + create_job("build_prio", githubuser, release, repo, email, prio_distro, prio_arch, post_jobs)
    return results


def find_stack(stack):
    # function to check if inserted stack is available
    
    global repositories
    available_stacks = ['cob_apps', 'cob_common', 'cob_driver', 'cob_extern', 'cob_simulation', 'cob3_intern', 'srs', 'interaid']
    # correct common mistakes
    stack = stack.lower()
    stack = stack.replace('-', '_')
    if stack in available_stacks:
        repositories = repositories[:] + [stack]
    else:
        print "<p><font color='#FF0000'>ERROR:"
        print "Stack <b>" + stack + " </b>could not be found. Please check spelling!</font>"


def stack_forked(githubuser, stack):
    # function to check if stack is forked on Github.com
    
    # get token from jenkins' .gitconfig file for private github forks
    gitconfig = open("/home/jenkins/.gitconfig", "r") 
    gitconfig = gitconfig.read()
    # extract necessary data
    regex = ".*\[github]\s*user\s*=\s*([^\s]*)\s*token\s*=\s*([^\s]*).*"
    gitinfo = re.match(regex, gitconfig, re.DOTALL)
    post = {'login' : gitinfo.group(1), 'token' : gitinfo.group(2)}
    fields = urllib.urlencode(post)
    
    path = "https://github.com/" + githubuser + "/" + stack + "/blob/master/Makefile"
    file1 = StringIO.StringIO()
    
    c = pycurl.Curl()
    c.setopt(pycurl.URL, path)
    c.setopt(pycurl.POSTFIELDS, fields)
    c.setopt(pycurl.WRITEFUNCTION, file1.write) # to avoid to show the called page
    c.perform()
    c.close
    if c.getinfo(pycurl.HTTP_CODE) == 200:
        return True
    else:
        print "ERRORCODE: ", c.getinfo(pycurl.HTTP_CODE)
        return False


def job_exists(job_name):
    # function to check if job already exists on jenkins
    
    path = '/job/' + job_name
    conn = httplib.HTTPConnection("cob-kitchen-server", 8080)
    conn.request('GET', path)
    response = conn.getresponse()
    if response.status == 302:
        return True
    else:
        return False
      
        
def create_job(job_type, githubuser, release, repo, email="", distro="", arch="", post_jobs=[]):
    # function to create pipeline config (config for first job of pipeline)
    
    if job_type == "pipe": 
        UNIVERSAL_CONFIG = open("cgi_pipestart_config.xml", "r+w")
        job_name = release + "__" + githubuser + "__" + repo + "__pipe"
    elif job_type == "build" or job_type == "build_prio":    
        UNIVERSAL_CONFIG = open("cgi_config.xml", "r+w")
        job_name = release + "__" + githubuser + "__" + repo + "__" + distro + "__" + arch
    else:    
        return "wrong job_type specified (this should never happen)"

    # replacing placeholder
    jenkins_config = UNIVERSAL_CONFIG.read()
    jenkins_config = jenkins_config.replace('---LABEL---', job_type)
    jenkins_config = jenkins_config.replace('---GITHUBUSER---', githubuser)
    jenkins_config = jenkins_config.replace('---EMAIL---', email)
    jenkins_config = jenkins_config.replace('---ROSRELEASE---', release)
    jenkins_config = jenkins_config.replace('---REPOSITORY---', repo)
    jenkins_config = jenkins_config.replace('---DISTRIBUTION---', distro)
    jenkins_config = jenkins_config.replace('---ARCHITECTURE---', arch)
    jenkins_config = jenkins_config.replace('---POSTJOBS---', ','.join(str(n) for n in post_jobs))

    # check if job already exists
    exists = job_exists(job_name)
    if not exists:
        # create new job
        path = '/createItem?name=' + job_name
    else:
        # update existing job
        path = '/job/' + job_name + '/config.xml'

    # send job config to jenkins API
    gitconfig = open("/home/jenkins/.gitconfig", "r") 
    gitconfig = gitconfig.read()
    # extract necessary data
    regex = ".*\[jenkins]\s*user\s*=\s*([^\s]*)\s*password\s*=\s*([^\s]*).*"
    gitinfo = re.match(regex, gitconfig, re.DOTALL)
    base64string = base64.encodestring('%s:%s' % (gitinfo.group(1), gitinfo.group(2))).strip()
    headers = {"Content-Type": "text/xml", "charset": "UTF-8", "Authorization": "Basic %s" % base64string }
    conn = httplib.HTTPConnection("cob-kitchen-server", 8080)
    conn.request('POST', path, jenkins_config, headers)
    response = conn.getresponse()
    conn.close
    if response.status != 200:
        if exists:
            return job_name + ": <big>FAILED</big> to update: %d %s<br>" %(response.status, response.reason)
        else:
            return job_name + ": <big>FAILED</big> to create: %d %s<br>" %(response.status, response.reason)
    else:
        if exists:
            return job_name + ": updated <big>SUCCESSFULLY</big><SMALL> with post_jobs: " + str(post_jobs) + "</SMALL><br>"
        else:
            return job_name + ": created <big>SUCCESSFULLY</big><SMALL> with post_jobs: " + str(post_jobs) + "</SMALL><br>"



main()    
