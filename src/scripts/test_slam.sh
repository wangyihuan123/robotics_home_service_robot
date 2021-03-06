#!/bin/sh

xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5

# gmapping_demo also works
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" &
#
#xterm  -e  " roslaunch gmapping slam_gmapping_pr2.launch" &

sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e   " roslaunch turtlebot_teleop keyboard_teleop.launch"