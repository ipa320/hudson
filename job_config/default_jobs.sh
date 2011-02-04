#!/bin/bash
./create_job.sh diamondback ipa-fmw cob_extern
./create_job.sh diamondback ipa-fmw cob_common
./create_job.sh diamondback ipa-fmw cob_driver
./create_job.sh diamondback ipa-fmw cob_simulation
./create_job.sh diamondback ipa-fmw cob_apps
./create_gazebo_job.sh diamondback ipa-fmw
