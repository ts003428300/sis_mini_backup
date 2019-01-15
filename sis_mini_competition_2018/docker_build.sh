#!/usr/bin/env sh
chmod +x catkin_ws/src/object_detection/src/object_detection.py
xhost +
docker build -t sis_competition -f Dockerfile .
#chmod +x catkin_ws/src/object_detection/src/object_detection.py
#source catkin_ws/devel/setup.bash
#cd catkin_ws && catkin_make
