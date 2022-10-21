cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
        LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)
file(GLOB WHEELCHAIR_PLANNER src/wheelchair_planner/*.cpp)

add_executable(wheelchair_planner src/exe/main.cpp ${WHEELCHAIR_PLANNER})
target_link_libraries(wheelchair_planner ${catkin_LIBRARIES} -lstdc++fs)
