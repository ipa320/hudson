#!/bin/bash

export ROS_MASTER_URI=http://localhost:22422
export SIMX=-r

# start cob_bringup in background 
roslaunch cob_bringup sim.launch &
sleep 60s
pid_bringup="$(jobs -p)" # get the job PID
echo "cob_bringup PID: " $pid_bringup                           # to be deleted

# start cob_script_server in background
roslaunch cob_script_server script_server.launch &
sleep 150s
# get the job PID
pid_script_server_raw="$( jobs -l | grep ]+ )"
sleep 1s
pid_script_server_raw=${pid_script_server_raw%% Running*}
sleep 1s
pid_script_server=${pid_script_server_raw##*]+ }
sleep 1s
echo "cob_script_server PID: " $pid_script_server                 # to be deleted

do_test(){
    #set test parameter for current component and target
    rosparam set /component_test/component "$1"
    rosparam set /component_test/command_topic /"$1"_controller/command
    rosparam set /component_test/state_topic /"$1"_controller/state
    rosparam set /component_test/target "$2"
    sleep 1s
    echo "Component: " $1 " / Target: " $2                       # to be deleted
    #write introduction for current test to test file
    echo "-------------------------------------------" >> $WORKSPACE/../component_test_result.txt
    echo "Component: " $1 " / Target: " $2 >> $WORKSPACE/../component_test_result.txt
    echo "-------------------------------------------" >> $WORKSPACE/../component_test_result.txt
    #start test and write results to text file
    test/trajectory_test.py >> $WORKSPACE/../component_test_result.txt
    sleep 1s
    # cat $WORKSPACE/../component_test_result.txt                  # to be deleted
}

#change to cob_component_test package
cd "$( rospack find cob_component_test )"/ros

#create/overwrite text file for test results
echo "TEST RESULTS" > $WORKSPACE/../component_test_result.txt
cat $WORKSPACE/../component_test_result.txt                      # to be deleted

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
do_test sdh cylclosed
do_test sdh spheropen
do_test head front
do_test head back

sleep 10s

# kill cob_script_server and cob_bringup with before stored PIDs
kill "$pid_script_server"
kill "$pid_bringup"

#evaluate test results
echo "SUMMARY: " >> $WORKSPACE/../component_test_result.txt
tests_no="$( grep -c ' * TESTS: ' $WORKSPACE/../component_test_result.txt )"
success_no="$( grep -c ' * RESULT: SUCCESS' $WORKSPACE/../component_test_result.txt )" 
errors_no="$( grep -c ' * ERRORS: 1 ' $WORKSPACE/../component_test_result.txt )"
failures_no="$( grep -c ' * FAILURES: 1 ' $WORKSPACE/../component_test_result.txt )"

if [ $success_no -eq $tests_no -a $tests_no -ne 0 ]; then
   result="SUCCESS"
else
   result="FAIL"
fi

echo " * TESTS: [""$tests_no""]" >> $WORKSPACE/../component_test_result.txt
echo " * SUCCESS: [""$success_no""]" >> $WORKSPACE/../component_test_result.txt
echo " * ERRORS: [""$errors_no""]" >> $WORKSPACE/../component_test_result.txt
echo " * FAILURES: [""$failures_no""]" >> $WORKSPACE/../component_test_result.txt
echo " * RESULT:" "$result" >> $WORKSPACE/../component_test_result.txt

sleep 15s
cat $WORKSPACE/../component_test_result.txt

if [ "$result" == "SUCCESS" ]; then
   exit 0
else
   exit 1
fi
