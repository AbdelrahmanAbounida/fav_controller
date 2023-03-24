cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
)
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  customPos.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/localizer.py
  nodes/path_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)