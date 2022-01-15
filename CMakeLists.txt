cmake_minimum_required(VERSION 3.5)
project(gripper_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_firmware REQUIRED)
find_package(std_srvs REQUIRED)
set(dependencies
  rclcpp
  std_srvs
  arm_firmware  
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

include_directories(
  include
  /usr/local/include/modbus/
  ${catkin_INCLUDE_DIRS}
)

add_library(robotiq_gripper_driver SHARED
  include/gripper_ros2/robotiq_gripper_driver.h 
  src/robotiq_gripper_driver.cpp
  )
target_link_libraries(robotiq_gripper_driver modbus)
ament_target_dependencies(robotiq_gripper_driver ${dependencies})

add_executable(gripper_ros2_node include/gripper_ros2/gripper_ros2_node.h src/gripper_ros2_node.cpp)
target_link_libraries(gripper_ros2_node robotiq_gripper_driver)
ament_target_dependencies(gripper_ros2_node ${dependencies})



install(TARGETS robotiq_gripper_driver
  gripper_ros2_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include

)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()






