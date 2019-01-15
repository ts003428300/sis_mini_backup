# Robot Navigation
This package contains one .srv file for the rosservice, two .py files, 
robot_navigate_node.py defines the service and demo.py demonstrate how to call the service in python.

# How to run
To run the demonstration, please put the robot in the middle of tag 4 and tag 6 and run with folowing command

`
mm_pi $ roslaunch sis_mobile navigation_pi.launch x:=1 y:=0.9 th:=0
`

`
docker $ roslaunch robot_navigation robot_navigation.launch
`

`
docker $ python ~/sis_mini_competition_2018/catkin_ws/src/robot_navigation/src/demo.py
`

The robot will start to move to the front of apriltags with id 1 with distance 40 cm if it sees tag 1.
