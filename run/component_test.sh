#!/bin/bash

export ROS_MASTER_URI=http://localhost:22422
export SIMX=-r

# start cob_bringup in background 
roslaunch cob_bringup sim.launch &
sleep 60s
pid_bringup="$(jobs -p)" # get the job PID

# start cob_script_server in background
roslaunch cob_script_server script_server.launch &
sleep 150s
# get the job PID
pid_script_server_raw="$( jobs -l | grep ]+ )"
pid_script_server_raw=${pid_script_server_raw%% Running*}
pid_script_server=${pid_script_server_raw##*]+ }


cd "$( rospack find cob_component_test )"/ros

# ARM TEST
# HOME POSITION
rosparam load launch/param_arm_test.yaml # load needed arm parameters for component_test 
sleep 1s
echo "TEST RESULTS
-------------------------------------------
ARM TEST / HOME:
-------------------------------------------" > $WORKSPACE/../component_test_result.txt
test/trajectory_test.py > $WORKSPACE/../component_test_result.txt # start component_test
sleep 1s

# PREGRASP POSITION
rosparam set /component_test/target "pregrasp" # change target parameter
sleep 1s
echo "-------------------------------------------" >> $WORKSPACE/../component_test_result.txt
echo "ARM TEST / PREGRASP:" >> $WORKSPACE/../component_test_result.txt
echo "-------------------------------------------" >> $WORKSPACE/../component_test_result.txt
test/trajectory_test.py >> $WORKSPACE/../component_test_result.txt # start component_test
sleep 1s

# TRAY TEST
rosparam load launch/param_tray_test.yaml #load parameters for tray
sleep 1s
echo "--------------------------------------------" >> $WORKSPACE/../component_test_result.txt
echo "TRAY TEST:" >> $WORKSPACE/../component_test_result.txt
echo "--------------------------------------------" >> $WORKSPACE/../component_test_result.txt
test/trajectory_test.py >> $WORKSPACE/../component_test_result.txt


# kill cob_script_server and cob_bringup with before stored PIDs
kill "$pid_script_server"
kill "$pid_bringup"

# 
echo "SUMMARY: " >> $WORKSPACE/../component_test_result.txt
tests_no="$( grep -c ' * TESTS: ' $WORKSPACE/../component_test_result.txt )"
success_no="$( grep -c ' * RESULT: SUCCESS' $WORKSPACE/../component_test_result.txt )" 
errors_no="$( grep -c ' * ERRORS: 1 ' $WORKSPACE/../component_test_result.txt )"
failures_no="$( grep -c ' * FAILURES: 1 ' $WORKSPACE/../component_test_result.txt )"

if [ $success_no -eq $tests_no -a $tests_no -ne '0']; then
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

if [ $result == "SUCCESS" ]; then
   exit 0
else
   exit 1
fi
