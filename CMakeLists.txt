cmake_minimum_required(VERSION 2.8.11)
project(rosserial_qt)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rosserial_msgs rosserial_python rosserial_server rostest std_msgs)

catkin_package()

install(
  DIRECTORY src/ros_lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

catkin_install_python(
  PROGRAMS scripts/make_library.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
