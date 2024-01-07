#!/bin/bash
catkin_make
. devel/setup.bash
sudo chmod +777 /dev/ttyUSB0
roslaunch au_crustcrawler_base base.launch
