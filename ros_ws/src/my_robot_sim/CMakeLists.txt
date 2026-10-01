cmake_minimum_required(VERSION 3.0.2)
project(my_robot_sim)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
