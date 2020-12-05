cmake_minimum_required(VERSION 2.8.3)
project(jetbot_hw)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package()

catkin_install_python(PROGRAMS scripts/ina_pub.py scripts/ina_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
