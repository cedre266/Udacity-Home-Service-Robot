#!/bin/sh
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="$(pwd)/my_robot/worlds/my_world.world"" &
sleep 5
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "source $(pwd)/../devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"

# $(rospack find add_markers)

#xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:="/home/workspace/catkin_ws/src/my_robot/worlds/my_world.world"" &
#sleep 5
#xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" &
#sleep 5
#xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
#sleep 5
#xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch"
