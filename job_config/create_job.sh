#!/usr/bin/env bash

# checking input parameters
if [ $# == 3 ] || [ $# == 4 ]; then
	ROSRELEASE=$1
	GITHUBUSER=$2
	REPOSITORY=$3
	EMAIL=$4
	JOBNAME=$ROSRELEASE\_\_$GITHUBUSER\_\_$REPOSITORY
	echo "Creating hudson config with"
	echo "ROS release            = " $ROSRELEASE
	echo "Github user name       = " $GITHUBUSER
	echo "Github repository name = " $REPOSITORY
	echo "User email             = " $EMAIL
	echo "Job name               = " $JOBNAME
else
	echo "ERROR: Wrong number of parameters"
	echo "Usage: create_job.sh ROSRELEASE GITHUBUSER REPOSITORY [EMAIL]"
	exit 1
fi

# checking for ROS release
if [[ "$ROSRELEASE" != "boxturtle" && "$ROSRELEASE" != "cturtle" && "$ROSRELEASE" != "diamondback" && "$ROSRELEASE" != "unstable" ]]; then
	echo "ERROR: no valid ROS release specified"
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
sudo sed -i "s/---GITHUBUSER---/$GITHUBUSER/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---REPOSITORY---/$REPOSITORY/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---ROSRELEASE---/$ROSRELEASE/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---EMAIL---/$EMAIL/g" /var/lib/hudson/jobs/$JOBNAME/config.xml
sudo sed -i "s/---JOBNAME---/$JOBNAME/g" /var/lib/hudson/jobs/$JOBNAME/config.xml

# change owner to hudson
sudo chown -R hudson.hudson /var/lib/hudson/jobs

echo ""
echo "Job $JOBNAME created."
echo "Restart Hudson to include new job."
