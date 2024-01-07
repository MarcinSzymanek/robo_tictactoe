#!/bin/bash
. ./devel/setup.bash
sudo chmod +777 /dev/ttyUSB0
roslaunch au_crustcrawler_base meta.launch
rosrun tic_tac_toebot robobrain_launcher.py
