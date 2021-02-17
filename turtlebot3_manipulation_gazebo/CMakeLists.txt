################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_gazebo)

# ROS 2 relies on the C++14 standard. Depending on what compiler you’re using, support for C++14 might not be enabled by default. Using gcc 5.3 (which is what is used on Ubuntu Xenial), we need to enable it explicitly, which we do by adding this line near the top of the file:
# The preferred way to work on all platforms is this:
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-std=c++11)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################

# Replace the find_package invocation with catkin and the COMPONENTS with: 
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
# find_package(gazebo REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Build
################################################################################
# link_directories(${GAZEBO_LIBRARY_DIRS}) 


# TODO
# target_link_libraries(target ${GAZEBO_LIBRARY_DIRS})  # IS GAZEBO AN AMENT PACKAGE OR NOT? IF SO, USE ament_target_dependencies() instead (TODO)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${GAZEBO_INCLUDE_DIRS}
#)
## The only directories that need to be manually included are local directories and dependencies that are not ament packages:
# include_directories(include ${GAZEBO_INCLUDE_DIRS})
## A better alternative is to specify include directories for each target individually, rather than including all the directories for all targets:
# target_include_directories(target include ${GAZEBO_INCLUDE_DIRS})


################################################################################
# Install
################################################################################
# For installation, catkin defines variables like CATKIN_PACKAGE_BIN_DESTINATION. With ament_cmake, we just give a path relative to the installation root, like bin for executables:

# install(DIRECTORY config launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################


################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
# Invoke ament_package instead but after all targets have been registered

ament_export_dependencies(gazebo_ros_pkgs)

ament_package()
