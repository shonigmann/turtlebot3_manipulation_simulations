cmake_minimum_required(VERSION 3.5)
project(turtlebot3_manipulation_simulations)

# ROS 2 relies on the C++14 standard. Depending on what compiler you’re using, support for C++14 might not be enabled by default. Using gcc 5.3 (which is what is used on Ubuntu Xenial), we need to enable it explicitly, which we do by adding this line near the top of the file:
# The preferred way to work on all platforms is this:
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Replace the find_package invocation with catkin and the COMPONENTS with: 
find_package(ament_cmake REQUIRED)

<export>
  <build_type>ament_cmake</build_type>
</export>

# catkin_metapackage()  # metapackages are no longer a special package type. can remove metapackage tags from package.xml (presumably also cmakelists
ament_package()  # Move and update the catkin_package invocation with: Invoke ament_package instead but after all targets have been registered.

