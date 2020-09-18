cmake_minimum_required(VERSION 3.0.2)
project(aip_xx2_description)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    config
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
