#!/bin/bash

export ROS_MASTER_URI=http://localhost:22422
export SIMX=-r

# start cob_bringup in background 
roslaunch cob_bringup sim.launch &
sleep 20s
pid_bringup="$(jobs -p)" # get the job PID

# start cob_script_server in background
roslaunch cob_script_server script_server.launch &
sleep 20s
# get the job PID
pid_script_server_raw="$( jobs -l | grep ]+ )"
pid_script_server_raw=${pid_script_server_raw%% Running*}
pid_script_server=${pid_script_server_raw##*]+ }


cd "$( rospack find cob_gazebo )"/ros/test

# ARM TEST
# HOME POSITION
rosparam load param_arm_test.yaml # load needed parameters for component_test 
sleep 1s
echo "TEST RESULTS" > component_test_result.txt
echo "-------------------------------------------" >> component_test_result.txt
echo "ARM TEST / HOME:" >> component_test_result.txt
echo "-------------------------------------------" >> component_test_result.txt
./component_test.py > component_test_result.txt # start component_test
sleep 1s

# PREGRASP POSITION
rosparam set /component_test/target "pregrasp"
sleep 1s
echo "-------------------------------------------" >> component_test_result.txt
echo "ARM TEST / PREGRASP:" >> component_test_result.txt
echo "-------------------------------------------" >> component_test_result.txt
./component_test.py >> component_test_result.txt # start component_test
sleep 1s

# TRAY TEST
rosparam load param_tray_test.yaml
sleep 1s
echo "--------------------------------------------" >> component_test_result.txt
echo "TRAY TEST:" >> component_test_result.txt
echo "--------------------------------------------" >> component_test_result.txt
./component_test.py >> component_test_result.txt


# kill cob_script_server and cob_bringup with before stored PIDs
kill "$pid_script_server"
kill "$pid_bringup"

# 
echo "SUMMARY: " >> component_test_result.txt
tests_no="$( grep -c ' * TESTS: ' component_test_result.txt )"
success_no="$( grep -c ' * RESULT: SUCCESS' component_test_result.txt )" 
errors_no="$( grep -c ' * ERRORS: 1 ' component_test_result.txt )"
failures_no="$( grep -c ' * FAILURES: 1 ' component_test_result.txt )"

if [ $success_no -eq $tests_no ]; then
   result="SUCCESS"
else
   result="FAIL"
fi

echo " * TESTS: [""$tests_no""]" >> component_test_result.txt
echo " * SUCCESS: [""$success_no""]" >> component_test_result.txt
echo " * ERRORS: [""$errors_no""]" >> component_test_result.txt
echo " * FAILURES: [""$failures_no""]" >> component_test_result.txt
echo " * RESULT:" "$result" >> component_test_result.txt

sleep 15s
cat component_test_result.txt

echo "[$result]" >&2
