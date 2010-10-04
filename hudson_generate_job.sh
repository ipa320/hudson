#!/usr/bin/env bash

# checking input parameters
# $1 GITHUBUSER
# $2 REPOSITORY
# $3 ROSRELEASE
if [ $# != 3 ]; then
	echo "ERROR: Wrong number of parameters"
	echo "Usage: hudson_generate_job.sh GITHUBUSER REPOSITORY ROSRELEASE"
	exit 1
else
	echo "Generating hudson config with"
	echo "Github user name       = " $1
	echo "Github repository name = " $2
	echo "ROS release            = " $3
fi

# create new job directory
sudo mkdir -p /var/lib/hudson/jobs/$2_$3_$1

# copy config file
sudo cp config.xml /var/lib/hudson/jobs/$2_$3_$1/config.xml

# generate config file
sudo sed -i "s/---GITHUBUSER---/$1/g" /var/lib/hudson/jobs/$2_$3_$1/config.xml
sudo sed -i "s/---REPOSITORY---/$2/g" /var/lib/hudson/jobs/$2_$3_$1/config.xml
sudo sed -i "s/---ROSRELEASE---/$3/g" /var/lib/hudson/jobs/$2_$3_$1/config.xml

# change owner to hudson
sudo chown -R hudson.hudson /var/lib/hudson/jobs
