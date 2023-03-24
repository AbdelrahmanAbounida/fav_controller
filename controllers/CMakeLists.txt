cmake_minimum_required(VERSION 3.0.2)
project(controllers)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  std_msgs
)

add_message_files(
  FILES
  config_msg.msg
)

# generate_messages(
#     DEPENDENCIES
#     std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/PidControlMinimal.cfg

 )
 generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  nodes/depth_setpoint.py
  nodes/depth_pid.py
  nodes/depth_calculator.py
  nodes/reconf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
