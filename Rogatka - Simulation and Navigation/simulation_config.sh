#!/bin/sh
cd ardupilot
git submodule init
git submodule update --recursive
./Tools/gittools/submodule-sync.sh
./waf configure --board sitl
./waf copter