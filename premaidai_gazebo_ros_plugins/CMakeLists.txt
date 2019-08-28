cmake_minimum_required(VERSION 2.8.3)
project(premaidai_gazebo_ros_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  gazebo_plugins
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES premaidai_gazebo_plugins
  CATKIN_DEPENDS gazebo_plugins
#   DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Declare a C++ library
add_library(PremaidAIPlugin src/PremaidAIPlugin.cc)
target_link_libraries(PremaidAIPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(TARGETS
  PremaidAIPlugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/plugins/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
