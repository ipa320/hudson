#!/bin/bash

# get the name of ROSRELEASE and GITHUBUSER from JOB_NAME
REPOSITORY=cob_apps
INTERSTAGE="${JOB_NAME%__*}"
GITHUBUSER="${INTERSTAGE#*__}"
RELEASE="${INTERSTAGE%__*}"


write_rosinstall(){
	STACK="$1"
	echo "- git: 
    local-name: $STACK
    uri: git@github.com:---GITHUBUSER---/$STACK.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
}

check_stack(){
	STACK="$1"
	user=`git config --global github.user`
	token=`git config --global github.token`
	wget --post-data "login=$user&token=$token" --spider https://github.com/"$GITHUBUSER"/"$STACK"/blob/master/Makefile --no-check-certificate 2> $WORKSPACE/../wget_response.txt
	return $(grep -c "200 OK" $WORKSPACE/../wget_response.txt)
}

# installing ROS release
sudo apt-get autoclean
sudo apt-get update
sudo apt-get install python-setuptools -y
sudo easy_install -U rosinstall
sudo apt-get install ros-$RELEASE-care-o-bot -y

# create .rosinstall file
echo "- other: {local-name: /opt/ros/---ROSRELEASE---/ros}
- other: {local-name: /opt/ros/---ROSRELEASE---/stacks}
" > $WORKSPACE/../$REPOSITORY.rosinstall

# get dependencies
rm $WORKSPACE/../$REPOSITORY.deps
wget https://github.com/ipa320/hudson/raw/master/run/"$REPOSITORY".deps -O $WORKSPACE/../$REPOSITORY.deps --no-check-certificate

echo ""
echo "--------------------------------------------------------------------------------"
echo "Checking dependencies for $REPOSITORY"
while read myline
do
  # check if stack is forked > true: include into .rosinstall file / false: check if it's reasonable to continue
  echo "Performing check on stack: $myline"
  check_stack $myline
  if [ $? != 0 ]; then
    write_rosinstall $myline
    echo "  INFO: Using stack $myline from $GITHUBUSER at github.com"
  else
    # repository is for sure just dependent on stack > continue 
    echo "  WARNING: Stack $myline not forked to $GITHUBUSER at github.com. Using release stack instead."
  fi
done < $WORKSPACE/../$REPOSITORY.deps
echo "--------------------------------------------------------------------------------"
echo ""

# delete unnecessary wget_response.txt
rm $WORKSPACE/../wget_response.txt

# generate .rosinstall file
sed -i "s/---GITHUBUSER---/$GITHUBUSER/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---ROSRELEASE---/$RELEASE/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---JOBNAME---/$JOB_NAME/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---REPOSITORY---/$REPOSITORY/g" $WORKSPACE/../$REPOSITORY.rosinstall

# perform clean rosinstall
rm $WORKSPACE/../.rosinstall
rosinstall $WORKSPACE/../ $WORKSPACE/../$REPOSITORY.rosinstall $WORKSPACE --delete-changed-uris

# setup ROS environment
. $WORKSPACE/../setup.sh

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
rosdep install cob_bringup -y
rosmake cob_bringup --skip-blacklist --profile

# cleanup gazebo tmp dir
#rm -rf /tmp/gazebo*

# export parameters
export SIMX=-r #no graphical output of gazebo
export ROBOT_ENV=ipa-kitchen

# create test directory
mkdir -p $WORKSPACE/test_results

# rostest cob3-1
export ROBOT=cob3-1
rm -rf ~/.ros/test_results # delete old rostest logs
rostest cob_bringup sim.launch # do the tests
rosrun rosunit clean_junit_xml.py # beautify xml files
mkdir -p $WORKSPACE/test_results/$ROBOT
cp -r ~/.ros/test_results/_hudson/*.xml $WORKSPACE/test_results/$ROBOT # copy test results

# rostest cob3-2
export ROBOT=cob3-2
rm -rf ~/.ros/test_results # delete old rostest logs
rostest cob_bringup sim.launch # do the tests
rosrun rosunit clean_junit_xml.py # beautify xml files
mkdir -p $WORKSPACE/test_results/$ROBOT
cp -r ~/.ros/test_results/_hudson/*.xml $WORKSPACE/test_results/$ROBOT # copy test results

# rostest desire
#export ROBOT=desire
#rostest cob_bringup sim.launch
