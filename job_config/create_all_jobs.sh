#!/bin/bash

# checking input parameters
if [ $# == 1 ] || [ $# == 2 ]; then
	GITHUBUSER=$1
	EMAIL=$2
	echo "Creating hudson jobs for"
	echo "Github user name = " $GITHUBUSER
	echo "User email = " $EMAIL
else
	echo "ERROR: Wrong number of parameters"
	echo "Usage: create_job.sh GITHUBUSER [EMAIL]"
	exit 1
fi

# diamondback jobs
./create_job.sh diamondback $GITHUBUSER cob_extern $EMAIL
./create_job.sh diamondback $GITHUBUSER cob_common $EMAIL
./create_job.sh diamondback $GITHUBUSER cob_driver $EMAIL
./create_job.sh diamondback $GITHUBUSER cob_simulation $EMAIL
./create_job.sh diamondback $GITHUBUSER cob_apps $EMAIL

# unstable jobs
./create_job.sh unstable $GITHUBUSER cob_extern $EMAIL
./create_job.sh unstable $GITHUBUSER cob_common $EMAIL
./create_job.sh unstable $GITHUBUSER cob_driver $EMAIL
./create_job.sh unstable $GITHUBUSER cob_simulation $EMAIL
./create_job.sh unstable $GITHUBUSER cob_apps $EMAIL

# gazebo jobs
./create_gazebo_job.sh diamondback $GITHUBUSER $EMAIL
./create_gazebo_job.sh unstable $GITHUBUSER $EMAIL
