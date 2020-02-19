##!/bin/sh
#
#xterm  -e  " source /home/kerry/catkin_ws/devel/setup.bash; roscore" &
#sleep 5
#xterm  -e  " roslaunch my_robot world.launch" &
#sleep 5
#xterm -e " roslaunch my_robot amcl.launch" &
#sleep 5
## either use keyboard or 2d nav goal from Rviz
#xterm -e   " rosrun teleop_twist_keyboard teleop_twist_keyboard.py"