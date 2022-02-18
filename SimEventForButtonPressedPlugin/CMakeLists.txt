cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

if (WIN32)
    add_definitions(-DBUILDING_DLL)
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set (sim_event_src
  EventSource.cc
  ExistenceEventSource.cc
  InRegionEventSource.cc
  JointEventSource.cc
  OccupiedEventSource.cc
  Region.cc
  SimEventsPlugin.cc
  SimStateEventSource.cc
)

set (sim_event_include
  EventSource.hh
  ExistenceEventSource.hh
  InRegionEventSource.hh
  JointEventSource.hh
  OccupiedEventSource.hh
  Region.hh
  SimEventsException.hh
  SimEventsPlugin.hh
  SimStateEventSource.hh
)

set (src
  EventSource.cc
  RegionEventBoxPlugin.cc
)

set (inc
  RegionEventBoxPlugin.hh
)


include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(SimEventForButtonPressedPlugin SHARED ${sim_event_src})
target_link_libraries(SimEventForButtonPressedPlugin gazebo_physics gazebo_msgs)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
