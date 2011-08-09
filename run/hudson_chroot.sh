#!/bin/bash

# ROSRELEASE, GITHUBUSER and REPOSITORY  !!!will be inserted automatically!!!


cd /tmp/workspace
WORKSPACE=/tmp/workspace/$REPOSITORY 
cp $WORKSPACE/.gitconfig ~/.gitconfig
cp -r $WORKSPACE/.ssh ~/
mkdir -p $WORKSPACE/test_results # create test_results directory
## create dummy test result file in case the script aborts before actual tests start
touch $WORKSPACE/test_results/no_test.xml
echo '<testsuite errors="1" failures="0" name="no_test" tests="1" time="0.01">
<testcase classname="NoTest.NoTest" name="no_test" time="0.01">
</testcase>
<system-out><![CDATA[Script aborted before actual tests could be started]]></system-out>
<system-err><![CDATA[]]></system-err>
</testsuite>' >> $WORKSPACE/test_results/no_test.xml


write_rosinstall(){
	STACK="$1"
	echo "- git: 
    local-name: $STACK
    uri: git://github.com/$GITHUBUSER/$STACK.git
    branch-name: master" >> $WORKSPACE/$REPOSITORY.rosinstall
}

check_stack(){
	STACK="$1"
	user=`git config --global github.user`
	token=`git config --global github.token`
	echo $user
	echo $token
	#wget --post-data "login=$user&token=$token" --spider https://github.com/"$GITHUBUSER"/"$STACK"/blob/master/Makefile --no-check-certificate 2> $WORKSPACE/wget_response.txt
	wget --post-data "login=$user&token=$token" --spider https://raw.github.com/"$GITHUBUSER"/"$STACK"/master/Makefile --no-check-certificate 2> $WORKSPACE/wget_response.txt
	cat $WORKSPACE/wget_response.txt
	return $(grep -c "200 OK" $WORKSPACE/wget_response.txt)
}

do_testing(){
	# sleep to finish running tests
	sleep 10

	# export parameters
	export ROBOT_ENV="$1"
	export ROBOT="$2"

	echo ""
	echo "start testing for $ROBOT in $ROBOT_ENV..."
	rm -rf ~/.ros/test_results # delete old rostest logs
	while read myline
	do
		rostest $myline
	done < $WORKSPACE/all.tests
	rosrun rosunit clean_junit_xml.py # beautify xml files
	mkdir -p $WORKSPACE/test_results
	for i in ~/.ros/test_results/_hudson/*.xml ; do mv "$i" "$WORKSPACE/test_results/$ROBOT-$ROBOT_ENV-`basename $i`" ; done # copy test results and rename with ROBOT

	# sleep to finish running tests
	sleep 10

	echo "...finished testing for $ROBOT in $ROBOT_ENV."
}

# installing ROS release
#sudo apt-get autoclean
sudo apt-get update
sudo apt-get install python-setuptools -y
sudo easy_install -U rosinstall
#sudo apt-get install ros-$RELEASE-ros -y
sudo apt-get install ros-$RELEASE-care-o-bot -y
sudo apt-get install bc -y                                                      ######################


# create .rosinstall file
echo "- other: {local-name: /opt/ros/---ROSRELEASE---/ros}
- other: {local-name: /opt/ros/---ROSRELEASE---/stacks}
- git: 
    local-name: $REPOSITORY
    uri: git://github.com/$GITHUBUSER/$REPOSITORY.git
    branch-name: master
" > $WORKSPACE/$REPOSITORY.rosinstall

# get dependencies
rm $WORKSPACE/$REPOSITORY.deps
wget https://github.com/ipa320/hudson/raw/master/run/"$REPOSITORY".deps -O $WORKSPACE/$REPOSITORY.deps --no-check-certificate

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
done < $WORKSPACE/$REPOSITORY.deps
echo "--------------------------------------------------------------------------------"
echo ""

# delete unnecessary wget_response.txt
rm $WORKSPACE/wget_response.txt

# generate .rosinstall file
sed -i "s/---GITHUBUSER---/$GITHUBUSER/g" $WORKSPACE/$REPOSITORY.rosinstall
sed -i "s/---ROSRELEASE---/$RELEASE/g" $WORKSPACE/$REPOSITORY.rosinstall
sed -i "s/---JOBNAME---/$JOB_NAME/g" $WORKSPACE/$REPOSITORY.rosinstall
sed -i "s/---REPOSITORY---/$REPOSITORY/g" $WORKSPACE/$REPOSITORY.rosinstall

# print rosinstall file
echo ""
echo "rosinstall file:"
cat $WORKSPACE/$REPOSITORY.rosinstall

# perform clean rosinstall
rm $WORKSPACE/.rosinstall
rosinstall $WORKSPACE $WORKSPACE/$REPOSITORY.rosinstall --delete-changed-uris

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

sleep 5

# installing dependencies and building
if [ $REPOSITORY == "cob3_intern" ]; then
    cd cob3_intern
    make ros-install
    make ros-skip-blacklist
else
    rosdep install $REPOSITORY -y
    rosmake $REPOSITORY --skip-blacklist --profile --status-rate=0
fi

# check if building is succesfull, otherwise don't perform test and exit
if [ $? != "0" ]; then
	echo "rosmake failed, skipping tests"
	exit 1
fi

# rostest
echo ""
echo "--------------------------------------------------------------------------------"
echo "Rostest for $REPOSITORY"

rm -rf ~/.ros/test_results # delete old rostest logs
rm -f $WORKSPACE/test_results/no_test.xml # delete dummy test file

if [ ! -s $WORKSPACE/all.tests ]; then
	echo "all.tests-file not found or empty, creating dummy test result file"
	# create dummy test result file
	touch $WORKSPACE/test_results/dummy_test.xml
	echo '<testsuite errors="0" failures="0" name="dummy_test" tests="1" time="0.01">
	<testcase classname="DummyTest.DummyTest" name="dummy_test" time="0.01">
	</testcase>
	<system-out><![CDATA[]]></system-out>
	<system-err><![CDATA[]]></system-err>
</testsuite>' >> $WORKSPACE/test_results/dummy_test.xml
else
	do_testing ipa-kitchen cob3-1
	do_testing ipa-kitchen cob3-2
	do_testing ipa-kitchen cob3-3
fi
echo "--------------------------------------------------------------------------------"
echo ""
