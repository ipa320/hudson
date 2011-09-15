#!/usr/bin/python

import cgi
import cgitb; cgitb.enable()
import pycurl
import re
import StringIO
import urllib
from subprocess import Popen, PIPE, STDOUT
import shlex
import os
import stat



def main():
    
    rosrelease = []
    repositories = []
    
    print "Content-Type: text/html\n\n"     # HTML is following

    keys = os.environ.keys()
    keys.sort()
    for k in keys:
        print "<li><b>%s:</b>\t\t%s<br>" %(k, os.environ[k]) 

    form = cgi.FieldStorage() # keys from HTML form

    # check if necessary keys (username & email) are available
    if "username" not in form or "email" not in form: # raise error if not
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
            if valid_stack(stack):
                repositories = repositories[:] + [stack]
    
    # printing planed job creations
    print "<p>Creating jobs to test:<br><ul>"    
    for stack in repositories:
        print "- ", stack, "<br>"
    print "</ul>"
    print "<hr>"

    # call spawn_jobs depending on 'delete' was selected
    if "delete" in form:
        print spawn_jobs(form["username"].value, form["email"].value, repositories, rosrelease, del_stacks=True)
    else:
        print spawn_jobs(form["username"].value, form["email"].value, repositories, rosrelease)

    print '<br>'
    print '<p><input type=button value="Back" onClick="history.back()">'


def spawn_jobs(githubuser, email, REPOSITORIES, ROSRELEASES, del_stacks=False):
    # function to spawn jobs

    results = """<p>JOB CREATION RESULTS<br>
====================<br>\n"""
    
    for release in ROSRELEASES:
        for repo in REPOSITORIES:
            results = results + "<br>"
            
            try:
                if not stack_forked(githubuser, repo):
                    results = results + "<b>" + repo + "</b>" + ": stack is not forked\n"
                    results = results + "Using 'ipa320' stack instead. If that isn't desired, fork " + repo + " on github.com!"
            except:
                results = results + "<b>Error: Checking whether stack %s is forked failed</b>"%repo
                results = results + "Using 'ipa320' stack instead. If that isn't desired, fork " + repo + " on github.com!"
                
            #TODO find right folder, output, try
            # call generate_prerelease.py with parameters: 'stack', 'rosdistro', 'githubuser', 'email'
            script = "generate_prerelease.py "
            parameters = "--stack %s --rosdistro %s --githubuser %s --email %s"%(repo, release, githubuser, email)
            if del_stacks:
                parameters = parameters + " --delete"
            bash_script = os.path.join("/home-local/jenkins", "bash_script.bash")
            with open(bash_script, "w") as f:
                f.write("""#!/bin/bash
                source /opt/ros/electric/setup.bash
                export ROS_PACKAGE_PATH=/home-local/jenkins/git/hudson:$ROS_PACKAGE_PATH
                export HOME=/home-local/jenkins
                echo "ROS_ROOT: " $ROS_ROOT "<br>"
                echo "ROS_PACKAGE_PATH: " $ROS_PACKAGE_PATH "<br>"
                roscd job_generation/scripts
                ./%s %s
                """%(script, parameters))
                os.chmod(bash_script, stat.S_IRWXU)
            p = Popen(bash_script, stdout=PIPE, stderr=STDOUT)
            out, err = p.communicate()
            results = results + "<br>" + out + "<br>"
            if err != None:
                results = results + "ERROR: " + err + "<br>"
    
    return results


def valid_stack(stack):
    # function to check if inserted stack is available
    available_stacks = ['cob_apps', 'cob_common', 'cob_driver', 'cob_extern', 'cob_simulation', 'cob3_intern', 'srs', 'interaid']
    # correct common mistakes
    stack = stack.lower()
    stack = stack.replace('-', '_')
    if stack in available_stacks:
        return True
    else:
        print "<p><font color='#FF0000'>ERROR:"
        print "Stack <b>" + stack + " </b>could not be found. Please check spelling!</font>"
        return False


def stack_forked(githubuser, stack):
    # function to check if stack is forked on Github.com
    
    # get token from jenkins' .gitconfig file for private github forks
    try:
        gitconfig = open("/home-local/jenkins/.gitconfig", "r") 
        gitconfig = gitconfig.read()
    except IOError as err:
        print "<b>ERROR" + err + "</b>"
        return False
    
    # extract necessary data
    regex = ".*\[github]\s*user\s*=\s*([^\s]*)\s*token\s*=\s*([^\s]*).*"
    gitinfo = re.match(regex, gitconfig, re.DOTALL)
    if gitinfo == None:
        print "<b>ERROR: No match found in 'gitconfig'</b>"
        raise
    post = {'login' : gitinfo.group(1), 'token' : gitinfo.group(2)}
    fields = urllib.urlencode(post)
    
    path = "https://github.com/" + githubuser + "/" + stack + "/blob/master/Makefile"
    file1 = StringIO.StringIO()
    
    try:
        c = pycurl.Curl()
        c.setopt(pycurl.URL, path)
        c.setopt(pycurl.POSTFIELDS, fields)
        c.setopt(pycurl.WRITEFUNCTION, file1.write) # to avoid to show the called page
        c.perform()
        c.close
    except:
        print "<b>ERROR: Problem occured while checking for 'Makefile'</b>"
        raise
        
    if c.getinfo(pycurl.HTTP_CODE) == 200:
        return True
    else:
        print "<b>ERRORCODE: ", c.getinfo(pycurl.HTTP_CODE), "</b>"
        return False


main()
