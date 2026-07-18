cmake_minimum_required(VERSION 3.0.2)
project(livox_elevation_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  gazebo_msgs
  visualization_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/local_elevation_mapper.py
  scripts/livox_sensor_follower.py
  scripts/synthetic_livox_cloud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
