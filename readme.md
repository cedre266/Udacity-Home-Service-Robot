# Udacity-Home-Service-Robot

## Overview
Last project in Udacity's Robotics Software Engineer Nanodegree.

The goal is to simulate a home service robot, that navigates into an indoor environment, goes to some place to pick up a virtual object, then moves to a second place to drop the virtual object off.

Video:

[![Demo Video](https://img.youtube.com/vi/nny7iTuVhYk/0.jpg)](https://www.youtube.com/watch?v=nny7iTuVhYk)

Made under ROS-Kinetic and Gazebo-7.

**Author: Cedric Perney**

## Installation
- Make sure to have a working installation of ROS-Kinetic
- Clone this git repository on your local computer, check/install any missing dependencies and compile the package:
```
cd catkin_ws/src
git clone https://github.com/cedre266/Udacity-Home-Service-Robot.git
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

## Usage
- Go to the right folder
```
cd catkin_ws/src
```
- Launch the shell script with the full process
```
./home_service.sh
```
- Several terminals should open, as well as Gazebo and Rviz. No additional intervention is required. Note that on the first time, Gazebo can be slow to open. If that is causing a problem, relaunching the shell script should solve it.

## Shell scripts
Multiple shell scripts were written that represent different steps in creating the overall simulation.

All shell scripts launch the turtlebot robot into a custom world.
- **`test_slam.sh`**: to test SLAM with the gmapping package and teleoperation. It can be used to create and save a map of the environment.
- **`test_navigation.sh`**: to test manual navigation with the AMCL package. It is possible to give navigation goals to the robot through Rviz.
- **`pick_objects.sh`**: to test sending navigation goals through code, calls the `pick_objects` node.
- **`add_markers.sh`**: to test creating virtual objects, calls the `add_markers_time` node, which spawns a virtual object, removes it after 5 seconds and respawn it in a new location after 5 more seconds.
- **`home_service.sh`**: the complete simulation. In addition to spawning the turtlebot in the custom world, launching the AMCL package for localization and opening Rviz for visualization, it also launches both the `add_markers` and `pick_objects` nodes that command the robot to its pickup and dropoff zones, using the ROS navigations stack, while moving the virtual object.

## How does it work
1. **Robot**: the robot used is a Turtlebot 2. It is a little mobile robot with a Kinect depth sensor that we use here as fake laser measurements to interface with the different packages for localization and mapping.
2. **Localization**: performed with the AMCL package (Adaptive Monte-Carlo Localization). This package is using a particle filter approach for localization with a known map. It generates particles that each represent a possible pose of the robot. At each iteration of the filter, the new laser measurement data is used to give a score to each particle. The ones with the best scores are kept while the other ones are resampled around them. This process, after enough iterations, should make all the particles converge around the true pose of the robot. The particles can sometimes struggle to converge if the environment does not have enough distinct features (such as parallel walls).
3. **Mapping**: performed with the Gmapping package, that is using the robot's laser scan data to perform SLAM (Simultaneous Localization And Mapping). By simultaneously tracking the pose of the robot and using the laser data to sense the robot's environment, the package is able to output a 2D occupancy grid map as well as an estimation of the robot's pose. Obtaining a map of the environment is crucial for the robot to perform global path planning and avoid any obstacle.
4. **Navigation**: the navigation capabilities rely on the ROS navigation stack. The map of the environment is used to determine the free and occupied spaces, namely where our robot is allowed to go or not. This is done by creating a cost map, by inflating all the obstacles by the width of the robot, in order to prevent the robot from trying to go to a place where it might touch obstacles. Path planning is then performed on this map. It relies on Dijkstra's algorithm in order to find the shortest path between the robot's current position and its goal. If a new obstacle is detected on the chosen trajectory, a new path is calculated to avoid it. Navigations goals are given here through the `pick_objects` node and correspond to the initial and final position of the virtual object.
5. **Virtual object**: as the Turtlebot has no object manipulation capability, we are simulating the picking up and dropping off of an object that does not exist in the simulated world (Gazebo), but we are representing it in Rviz, which is only for visualization, hence the word virtual. That virtual object is created at its initial position, deleted when the robot reaches the pick up location, and finally respawned at its final/drop off location when the robot reaches it. All of this is done in the `add_markers` node.

## Related documentation
- **Gmapping package**: http://wiki.ros.org/gmapping
- **AMCL package**: http://wiki.ros.org/amcl
- **ROS Navigation stack**: http://wiki.ros.org/navigation
- **Turtlebot**: http://wiki.ros.org/turtlebot_gazebo
