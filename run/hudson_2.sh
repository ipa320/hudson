#!/bin/bash

# get the name of REPOSITORY and GITHUBUSER from JOB_NAME
REPOSITORY="${JOB_NAME##*--}"
INTERSTAGE="${JOB_NAME%--*}"
GITHUBUSER="${INTERSTAGE#*--}"

write_rosinstall(){
	STACK="$1"
	echo "- git: 
    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/$STACK
    uri: git://github.com/---GITHUBUSER---/$STACK.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
}

check_stack(){
	STACK="$1"
	wget --spider https://github.com/"$GITHUBUSER"/"$STACK"/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt
}


# checking for ROS release
if [ $# != 1 ]; then
	echo "ERROR: no ROS release specified"
	exit 1
elif [ $1 = "boxturtle" ]; then
	RELEASE=boxturtle
elif [ $1 = "cturtle" ]; then
	RELEASE=cturtle
elif [ $1 = "unstable" ]; then
	RELEASE=unstable
else
	echo "ERROR: no valid ROS release specified"
	exit 1
fi

# installing ROS release
sudo apt-get update
sudo apt-get install ros-$RELEASE-care-o-bot -y

# get .rosinstall file
#wget https://github.com/ipa320/hudson/raw/master/run/empty.rosinstall -O $WORKSPACE/../$REPOSITORY.rosinstall --no-check-certificate

# create .rosinstall file
echo "- other: {local-name: /opt/ros/---ROSRELEASE---/ros}
- other: {local-name: /opt/ros/---ROSRELEASE---/stacks}
" > $WORKSPACE/../$REPOSITORY.rosinstall


# checking dependencies and writing in .rosinstall file
case "$REPOSITORY" in
	cob_extern|cob_common)
		# check if stack is forked > true include into .rosinstall file / false use release
		check_stack $REPOSITORY
#wget --spider https://github.com/"$GITHUBUSER"/"$REPOSITORY"/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt 
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall $REPOSITORY
#echo "- git: 
#    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/---REPOSITORY---
#    uri: git://github.com/---GITHUBUSER---/---REPOSITORY---.git
#    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "Stack $REPOSITORY not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
	;;
	cob_apps)
		check_stack cob_apps
#wget --spider https://github.com/"$GITHUBUSER"/cob_apps/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall cob_apps
#echo "- git: 
#    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/cob_apps
#    uri: git://github.com/---GITHUBUSER---/cob_apps.git
#    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "Stack cob_apps not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
	cob_simulation)
		wget --spider https://github.com/"$GITHUBUSER"/cob_simulation/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "- git: 
    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/cob_simulation
    uri: git://github.com/---GITHUBUSER---/cob_simulation.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "Stack cob_simulation not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
	cob_driver)
		wget --spider https://github.com/"$GITHUBUSER"/cob_driver/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "- git: 
    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/cob_driver
    uri: git://github.com/---GITHUBUSER---/cob_driver.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "Stack cob_driver not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
		wget --spider https://github.com/"$GITHUBUSER"/cob_extern/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "- git: 
    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/cob_extern
    uri: git://github.com/---GITHUBUSER---/cob_extern.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "Stack cob_extern not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
		wget --spider https://github.com/"$GITHUBUSER"/cob_common/blob/master/stack.xml --no-ckeck-certificate 2> $WORKSPACE/../wget_response.txt
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "- git: 
    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/cob_common
    uri: git://github.com/---GITHUBUSER---/cob_common.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			echo "Stack cob_common not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
	;;
esac

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


