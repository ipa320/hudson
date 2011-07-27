#!/bin/bash

export ROS_MASTER_URI=http://localhost:22422
export SIMX=-r

# start cob_bringup in background 
roslaunch cob_bringup sim.launch &
sleep 100s
pid_bringup="$(jobs -p)" # get the job PID

# start cob_script_server in background
roslaunch cob_script_server script_server.launch &
sleep 100s
# get the job PID
pid_script_server_raw="$( jobs -l | grep ]+ )"
sleep 1s
pid_script_server_raw=${pid_script_server_raw%% Running*}
sleep 1s
pid_script_server=${pid_script_server_raw##*]+ }

do_test(){
    #set test parameter for current component and target
    rosparam set /component_test/component "$1"
    rosparam set /component_test/command_topic /"$1"_controller/command
    rosparam set /component_test/state_topic /"$1"_controller/state
    rosparam set /component_test/target "$2"
    sleep 1s
    rm -rf ~/.ros/test_results          # delete old rostest logs
    test/trajectory_test.py             # start test
    rosrun rosunit clean_junit_xml.py   # beautify xml files
    for i in ~/.ros/test_results/_hudson/*.xml # copy test results and rename with ROBOT
        do
            sed -i "s/rostest.TEST-component_test/$ROBOT-rostest.TEST-$1-$2-component_test/g" "$i"
            mv "$i" "$WORKSPACE/test_results/$ROBOT-$ROBOT_ENV-$1-$2-`basename $i`"
    done 
    sleep 1s
}

#change to cob_component_test package
cd "$( rospack find cob_component_test )"/ros

#set static test parameters
rosparam set /component_test/wait_time 10.0
rosparam set /component_test/error_range 1.0

#tests for each component
do_test arm home
do_test arm pregrasp
do_test tray down
do_test tray up
do_test torso front
do_test torso right
do_test sdh spherclosed
do_test sdh cylopen
do_test sdh home
do_test head front
do_test head back

sleep 5s

# kill cob_script_server and cob_bringup with before stored PIDs
kill "$pid_script_server"
kill "$pid_bringup"

sleep 15s
