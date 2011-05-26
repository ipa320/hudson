#!/bin/bash

# get the job parameters (DISTRO, ARCH, ROSRELEASE, REPOSITORY and GITHUBUSER) from JOB_NAME
DISTRO="${JOB_NAME%%__*}"
REPOSITORY="${JOB_NAME##*__}"
INTERSTAGE1="${JOB_NAME%__*}" #cut off REPOSITORY
INTERSTAGE2="${INTERSTAGE1#*__}" #cut off DISTRO
RELEASE="${INTERSTAGE2#*__}"
ARCH="${INTERSTAGE2%__*}"

DISTRO_TGZ=${DISTRO}-base.tgz


sudo apt-get install pbuilder
[ -e ${DISTRO_TGZ} ] || 
{
    sudo pbuilder --create --distribution ${DISTRO} --othermirror "deb http://code.ros.org/packages/ros/ubuntu ${DISTRO} main" --basetgz ${DISTRO_TGZ} --components "main restricted universe multiverse" --extrapackages "wget lsb-release debhelper"
}
