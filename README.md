# Gazebo tutorial
A tutorial for simulation using gazebo and the turtlebot model. This repository starts a simulation of the turtle bot
in the gazebo simulation environment 

## Dependencies
- [Robot Operating System (ROS Kinetic)](http://wiki.ros.org/kinetic/Installation) (middleware for robotics),
- turlebot sim package. You can install it by running the command, `sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers`

## Standard build and run using roslaunch
```
cd <path to catkin workspace>/src
git clone --recursive https://github.com/ashwinvk94/turtlebot_sim
cd ../..
catkin_make
source devel/setup.bash
roslaunch turtlebot_sim walker.launch
```
The bag files will be stored in the directory named 'rosbag'

In order to launch the cimulation without rosbag, run `roslaunch turtlebot_sim walker.launch bagrecord:=false`

## Running and inspecting rosbag
Start roscore in a new terminal
```
cd <path to catkin workspace>/src/turtlebot_sim/rosbag
rosbag play <name_of _bagfile>.bag
rostopic list
```

## Cpplint check
Execute the following commands in a new terminal to run cpplint
```
cd  <path to repository>
cpplint $( find . -name \*.hpp -or -name \*.cpp )
```

## Cppcheck chec
Execute the following commands in a new terminal to run cppcheck
```
cd <path to repository>
cppcheck --enable=all --std=c++11 -I ../../devel/include/ -I ../../../../../../../opt/ros/kinetic/include/ -I ../../../../../../../usr/include/ --check-config --suppress=missingIncludeSystem $( find . -name *.cpp)
