#!/bin/bash

# get the name of ROSRELEASE and GITHUBUSER from JOB_NAME
REPOSITORY=cob_apps
INTERSTAGE="${JOB_NAME%__*}"
GITHUBUSER="${INTERSTAGE#*__}"
INTERSTAGE2="${INTERSTAGE%__*}"
RELEASE="${INTERSTAGE2#*__}"

# installing ROS release
sudo apt-get update
sudo apt-get install python-setuptools -y
sudo easy_install -U rosinstall
sudo apt-get install ros-$RELEASE-care-o-bot -y

# get .rosinstall file
#cp /home/hudson/$REPOSITORY.rosinstall $WORKSPACE/../$REPOSITORY.rosinstall
wget https://github.com/ipa320/hudson/raw/master/run/$REPOSITORY.rosinstall -O $WORKSPACE/../$REPOSITORY.rosinstall --no-check-certificate

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

echo ""
echo "-------------------------------------------------------"
echo "==> RELEASE =" $RELEASE
echo "==> WORKSPACE =" $WORKSPACE
echo "==> ROS_ROOT =" $ROS_ROOT
echo "==> ROS_PACKAGE_PATH =" $ROS_PACKAGE_PATH
echo "-------------------------------------------------------"
echo ""

# installing dependencies and building
rosdep install $cob_bringup
rosmake cob_bringup --skip-blacklist

# export parameters
export SIMX=-r #no graphical output of gazebo
export ROBOT_ENV=ipa-kitchen

# rostest
export ROBOT=cob3-1
rostest cob_bringup sim.launch
export ROBOT=cob3-2
rostest cob_bringup sim.launch
