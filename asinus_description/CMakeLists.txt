cmake_minimum_required(VERSION 3.8)
project(asinus_description)

# Suppress catkin-related warnings for ROS 2 workspace
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT CACHE)
endif()
if(DEFINED CATKIN_SYMLINK_INSTALL)
  unset(CATKIN_SYMLINK_INSTALL CACHE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY
    config
    launch
    asinus
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)



ament_package()