#!/bin/sh

xterm  -e  " source catkin_ws/devel/setup.bash; roscore" &
sleep 3

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5

xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5

xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

# debug
#rosrun pick_objects pick_objects _param:=pick_objects
xterm  -e  " rosrun pick_objects pick_objects _param:=pick_objects"
