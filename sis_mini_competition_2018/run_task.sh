#!/bin/bash

set -e

source ~/sis_mini_competition_2018/catkin_ws/devel/setup.sh

roslaunch sis_arm_planning master_task.launch

#roslaunch object_detection open_realsense_and_vis.launch


