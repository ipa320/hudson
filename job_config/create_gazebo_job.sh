#!/usr/bin/env bash

# checking input parameters
# $1 GITHUBUSER
# $2 REPOSITORY
# $3 ROSRELEASE
if [ $# == 1 ] || [ $# == 2 ]; then
	echo "Creating hudson config with"
	echo "Github user name       = " $1
	echo "User email             = " $2
	JOBNAME=gazebo-test__$1
else
	echo "ERROR: Wrong number of parameters"
	echo "Usage: create_gazebo_job.sh GITHUBUSER [EMAIL]"
	exit 1
fi

# check, if job already exists
if [ -d /var/lib/hudson/jobs/$JOBNAME ]; then
	read -p "Job already exists, overwriting configuration? (y/N)"
	if [[ $REPLY = [yY] ]]; then
		echo "Overwriting job $JOBNAME"
	else
		echo "Aborting..."
		exit 1
	fi
fi

# create new job directory
sudo mkdir -p /var/lib/hudson/jobs/$JOBNAME

# copy config file
sudo cp config.xml /var/lib/hudson/jobs/$JOBNAME/config.xml

# generate config file
sudo sed -i "s/---GITHUBUSER---/$1/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---REPOSITORY---/$2/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---EMAIL---/$4/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---JOBNAME---/$JOBNAME/g" /var/lib/hudson/jobs/$JOBNAME/config.xml

# change owner to hudson
sudo chown -R hudson.hudson /var/lib/hudson/jobs

echo ""
echo "Job $JOBNAME created."
echo "Restart Hudson to include new job."
