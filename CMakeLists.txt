cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/turtlebot_sim.cpp
# )
add_compile_options(-std=c++11)


include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(turtlebot_sim
		src/collision_avoidance.cpp
		src/main.cpp)
target_link_libraries(turtlebot_sim ${catkin_LIBRARIES})

