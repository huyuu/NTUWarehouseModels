cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ActorAvoidingObstaclesPlugin SHARED ActorAvoidingObstaclesPlugin.hh ActorAvoidingObstaclesPlugin.cc AStarPathPlanner.hh AStarPathPlanner.cc Node.hh Node.cc)
target_link_libraries(ActorAvoidingObstaclesPlugin ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
