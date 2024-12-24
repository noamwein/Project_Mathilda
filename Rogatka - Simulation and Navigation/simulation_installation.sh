#!/bin/sh
sudo apt update
sudo apt upgrade
git clone https://github.com/ArduPilot/ardupilot.git
ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y