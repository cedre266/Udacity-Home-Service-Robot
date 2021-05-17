#!/bin/sh
cd $(pwd)/..; catkin_make
xterm -e "source /home/workspace/catkin_ws/devel/set_up.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/set_up.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/set_up.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/set_up.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
