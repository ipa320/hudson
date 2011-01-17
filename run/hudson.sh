#!/bin/bash

STACKS="
cob_apps
cob_common
cob_driver
cob_extern
cob_simulation
"

# get the name of REPOSITORY and GITHUBUSER from JOB_NAME
REPOSITORY="${JOB_NAME##*--}"
INTERSTAGE="${JOB_NAME%--*}"
GITHUBUSER="${INTERSTAGE#*--}"

# checking for ROS release
if [ $# != 1 ]; then
	echo "ERROR: no ROS release specified"
	exit 1
elif [ $1 = "boxturtle" ]; then
	RELEASE=boxturtle
elif [ $1 = "latest" ]; then
	RELEASE=latest
elif [ $1 = "cturtle" ]; then
	RELEASE=cturtle
else
	echo "ERROR: no valid ROS release specified"
	exit 1
fi

# installing ROS release
sudo apt-get update
sudo apt-get install ros-$RELEASE-care-o-bot -y

# get .rosinstall file
#cp /home/hudson/$REPOSITORY.rosinstall $WORKSPACE/../$REPOSITORY.rosinstall
wget https://github.com/ipa-fmw/hudson/raw/master/run/$REPOSITORY.rosinstall -O $WORKSPACE/$REPOSITORY.rosinstall --no-check-certificate

# generate .rosinstall file
sed -i "s/---GITHUBUSER---/$GITHUBUSER/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---ROSRELEASE---/$RELEASE/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---JOBNAME---/$JOB_NAME/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---REPOSITORY---/$REPOSITORY/g" $WORKSPACE/../$REPOSITORY.rosinstall

# perform clean rosinstall
rm $WORKSPACE/.rosinstall
rm $WORKSPACE/../setup.sh
rosinstall $WORKSPACE $WORKSPACE/../$REPOSITORY.rosinstall $WORKSPACE --delete-changed-uris

# setup ROS environment
. $WORKSPACE/setup.sh

# define amount of ros prozesses during build for multi-prozessor machines
COUNT=$(cat /proc/cpuinfo | grep 'processor' | wc -l)
COUNT=$(echo "$COUNT*2" | bc)
export ROS_PARALLEL_JOBS=-j$COUNT

#build farm stuff
#export PATH=/usr/lib/ccache/:$PATH
#export DISTCC_HOSTS='localhost distcc@hektor.ipa.fhg.de/8 distcc@chaos.ipa.fhg.de/4'
#export CCACHE_PREFIX=distcc
#export ROS_PARALLEL_JOBS=-j28

echo ""
echo "-------------------------------------------------------"
echo "==> RELEASE =" $RELEASE
echo "==> WORKSPACE =" $WORKSPACE
echo "==> ROS_ROOT =" $ROS_ROOT
echo "==> ROS_PACKAGE_PATH =" $ROS_PACKAGE_PATH
echo "-------------------------------------------------------"
echo ""

# installing dependencies and building
rosdep install $REPOSITORY
rosmake $REPOSITORY --skip-blacklist


