#!/bin/sh
# Launch Turtlebot in custom world
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="$(pwd)/my_robot/worlds/my_world.world"" &
sleep 5
# Launch AMCL node and use map of custom world
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:="$(pwd)/map/my_map.yaml" initial_pose_a:=-1.57" &
sleep 5
# Launch Rviz set-up for navigation
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch"
