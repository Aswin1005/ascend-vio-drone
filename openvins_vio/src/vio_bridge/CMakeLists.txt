cmake_minimum_required(VERSION 3.0.2)
project(vio_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  mavros_msgs
  geographic_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vio_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
