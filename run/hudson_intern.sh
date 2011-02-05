#!/bin/bash

# get the name of ROSRELEASE, GITHUBUSER and REPOSITORY from JOB_NAME
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
	return $(grep -c "200 OK" $WORKSPACE/../wget_response.txt)
}

# installing ROS release
sudo apt-get update
sudo apt-get install python-setuptools -y
sudo easy_install -U rosinstall
sudo apt-get install ros-$RELEASE-care-o-bot -y


# create .rosinstall file
echo "- other: {local-name: /opt/ros/---ROSRELEASE---/ros}
- other: {local-name: /opt/ros/---ROSRELEASE---/stacks}
" > $WORKSPACE/../$REPOSITORY.rosinstall


# checking dependencies and writing in .rosinstall file
write_rosinstall cob3_intern



# check if stack is forked > true: include into .rosinstall file / false: check if it's reasonable to continue
check_stack cob_extern
if [ $? != 0 ]; then
	write_rosinstall cob_extern
else
	# repository is for sure just dependent on stack > continue 
	echo "WARNING: Stack cob_extern not forked to $GITHUBUSER on github.com. Using release stack instead."
fi

check_stack cob_common
if [ $? != 0 ]; then
	write_rosinstall cob_common
else
	# repository is for sure just dependent on stack > continue
	echo "WARNING: Stack cob_common not forked to $GITHUBUSER on github.com. Using release stack instead."
fi

check_stack cob_driver
if [ $? != 0 ]; then
	write_rosinstall cob_driver
else
	echo "WARNING: Stack cob_driver not forked to $GITHUBUSER on github.com. Using release stack instead."
fi

check_stack cob_simulation
if [ $? != 0 ]; then
	write_rosinstall cob_simulation
else
	echo "WARNING: Stack cob_simulation not forked to $GITHUBUSER on github.com. Using release stack instead."
fi

check_stack cob_apps
if [ $? != 0 ]; then
	write_rosinstall cob_apps
else
	echo "WARNING: Stack cob_apps not forked to $GITHUBUSER on github.com. Using release stack instead."
fi

# delete unnecessary wget_response.txt
rm $WORKSPACE/../wget_response.txt

# generate .rosinstall file
sed -i "s/---GITHUBUSER---/$GITHUBUSER/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---ROSRELEASE---/$RELEASE/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---JOBNAME---/$JOB_NAME/g" $WORKSPACE/../$REPOSITORY.rosinstall
sed -i "s/---REPOSITORY---/$REPOSITORY/g" $WORKSPACE/../$REPOSITORY.rosinstall

# perform clean rosinstall
rm $WORKSPACE/.rosinstall
rosinstall $WORKSPACE $WORKSPACE/../$REPOSITORY.rosinstall $WORKSPACE --delete-changed-uris

# setup ROS environment
. $WORKSPACE/setup.bash

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
make ros-install
make ros-skip-blacklist
