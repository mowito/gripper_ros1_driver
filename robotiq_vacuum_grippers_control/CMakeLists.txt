# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html

cmake_minimum_required(VERSION 3.5)
project(robotiq_vacuum_grippers_control)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

###################################
## catkin message generation     ##
###################################

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotiqVacuumGrippers_robot_input.msg"
  "msg/RobotiqVacuumGrippers_robot_output"
  "srv/AddThreeInts.srv"
  "srv/GripperCmd.srv"
 )
