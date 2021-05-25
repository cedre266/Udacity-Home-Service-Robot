#!/bin/sh
# Launch Turtlebot in custom world
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="$(pwd)/my_robot/worlds/my_world.world"" &
sleep 5
# Launch SLAM node
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
# Launch Rviz set-up for navigation
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
# Launch Turtlebot teleoperation node
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
