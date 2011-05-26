#!/usr/bin/env bash

# checking input parameters
if [ $# == 5 ] || [ $# == 6 ]; then
	DISTRO=$1
	ARCH=$2
	ROSRELEASE=$3
	REPOSITORY=$4
	GITHUBUSER=$5
	EMAIL=$6
	JOBNAME=$DISTRO\_\_$ARCH\_\_$ROSRELEASE\_\_$REPOSITORY
	echo "Creating hudson config with"
	echo "Ubuntu distribution    = " $DISTRO
	echo "Architecture           = " $ARCH
	echo "ROS release            = " $ROSRELEASE
	echo "Github repository name = " $REPOSITORY
	echo "Github user name       = " $GITHUBUSER
	echo "User email             = " $EMAIL
	echo "Job name               = " $JOBNAME
else
	echo "ERROR: Wrong number of parameters"
	echo "Usage: create_debbuild_job.sh DISTRO ARCH ROSRELEASE REPOSITORY GITHUBUSER [EMAIL]"
	exit 1
fi

# checking for Ubuntu distribution
if [[ "$DISTRO" != "jaunty" && "$DISTRO" != "karmic" && "$DISTRO" != "lucid" && "$DISTRO" != "maverick" ]]; then
	echo "ERROR: no valid Ubuntu distribution specified"
	echo "Supported distributions: jaunty karmic lucid maverick"
	exit 1
fi

# checking for architecture
if [[ "$ARCH" != "i386" && "$ARCH" != "amd64" ]]; then
	echo "ERROR: no valid architecture specified"
	echo "Supported architectures: i386 amd64"
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
sudo cp config_debbuild.xml /var/lib/hudson/jobs/$JOBNAME/config_debbuild.xml

# generate config file
sudo sed -i "s/---GITHUBUSER---/$GITHUBUSER/g" /var/lib/hudson/jobs/$JOBNAME/config_debbuild.xml
sudo sed -i "s/---REPOSITORY---/$REPOSITORY/g" /var/lib/hudson/jobs/$JOBNAME/config_debbuild.xml
sudo sed -i "s/---ROSRELEASE---/$ROSRELEASE/g" /var/lib/hudson/jobs/$JOBNAME/config_debbuild.xml
sudo sed -i "s/---EMAIL---/$EMAIL/g" /var/lib/hudson/jobs/$JOBNAME/config_debbuild.xml
sudo sed -i "s/---JOBNAME---/$JOBNAME/g" /var/lib/hudson/jobs/$JOBNAME/config_debbuild.xml

# change owner to hudson
sudo chown -R hudson.hudson /var/lib/hudson/jobs

echo ""
echo "Job $JOBNAME created."
echo "Restart Hudson to include new job."

