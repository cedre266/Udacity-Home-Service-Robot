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
- **test_slam.sh**: to test SLAM with the gmapping package and teleoperation. It can be used to create and save a map of the environment.
- **test_navigation.sh**: to test manual navigation with the AMCL package. It is possible to give navigation goals to the robot through Rviz.
- **pick_objects.sh**: to test sending navigation goals through code, calls the "pick_objects" node.
- **add_markers.sh**: to test creating virtual objects, calls the "add_markers" node.
- **home_service.sh**: the complete simulation. In addition to spawning the turtlebot in the custom world, launching the AMCL package for navigation and opening Rviz for visualization, it also launches both the "add_markers" and "pick_objects" nodes that command the robot to its pickup and dropoff zones while moving the virtual object.

## Related documentation
- **Gmapping package**: http://wiki.ros.org/gmapping
- **AMCL package**: http://wiki.ros.org/amcl
