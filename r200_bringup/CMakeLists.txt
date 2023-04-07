cmake_minimum_required(VERSION 2.8.3)
project(r200_bringup)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(
  DIRECTORY launch config include scrips
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
