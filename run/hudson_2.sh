#!/bin/bash

# get the name of REPOSITORY and GITHUBUSER from JOB_NAME
REPOSITORY="${JOB_NAME##*__}"
INTERSTAGE="${JOB_NAME%__*}"
GITHUBUSER="${INTERSTAGE#*__}"
RELEASE="${INTERSTAGE%__*}"

write_rosinstall(){
	STACK="$1"
	echo "- git: 
    local-name: /home/hudson/---ROSRELEASE---/---GITHUBUSER---/---JOBNAME---/$STACK
    uri: git://github.com/---GITHUBUSER---/$STACK.git
    branch-name: master" >> $WORKSPACE/../$REPOSITORY.rosinstall
}

check_stack(){
	STACK="$1"
	wget --spider https://github.com/"$GITHUBUSER"/"$STACK"/blob/master/stack.xml --no-check-certificate 2> $WORKSPACE/../wget_response.txt
}

check_abort(){
	STACK="$1"
	if [ $REPOSITORY == $STACK ]; then
		# if the repository isn't forked, it's senseless to continue
		echo "ERROR: Stack $STACK not forked to $GITHUBUSER on github.com. Aborting..."
		exit 1
	else
		# if the repository is just dependent on this stack 
		echo "WARNING: Stack $STACK not forked to $GITHUBUSER on github.com. Using release stack instead."
	fi
}

# installing ROS release
sudo apt-get update
sudo apt-get install python-setuptools -y
sudo easy_install -U rosinstall
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
		# check if stack is forked > true: include into .rosinstall file / false: abort
		check_stack $REPOSITORY
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall $REPOSITORY
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			# it is senseless to continue building
			echo "ERROR: Stack $REPOSITORY not forked to $GITHUBUSER on github.com. Aborting..."
			exit 1
		fi
	;;
	cob_apps)
		check_stack cob_apps
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall cob_apps
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			# it is senseless to continue building
			echo "ERROR: Stack cob_apps not forked to $GITHUBUSER on github.com. Aborting..."
			exit 1
		fi
	;&
	cob_simulation)
		# check if stack is forked > true: include into .rosinstall file / false: check if it's reasonable to continue
		check_stack cob_simulation
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall cob_simulation
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			check_abort cob_simulation
		fi
	;&
	cob_driver)
		check_stack cob_driver
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall cob_driver
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			check_abort cob_driver
		fi

		check_stack cob_extern
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall cob_extern
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			# repository is for sure just dependent on stack > continue 
			echo "Stack cob_extern not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi

		check_stack cob_common
		if [ "grep -c "200 OK" $WORKSPACE/../wget_response.txt" != 0 ]; then
			write_rosinstall cob_common
		elif [ "grep -c "404 Not Found" $WORKSPACE/../wget_response.txt" != 0 ]; then
			# repository is for sure just dependent on stack > continue
			echo "Stack cob_common not forked to $GITHUBUSER on github.com. Using release stack instead."
		fi
	;;
esac

# delete unnecessary wget_response.txt
rm $WORKSPACE/../wget_response.txt

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


