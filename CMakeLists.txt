cmake_minimum_required(VERSION 3.8)
project(cpp_parameter_event_handler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenIGTLink REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action)
  # tf2_geometry_msgs
  # tf2_ros
  # moveit_msgs
  # moveit_core
  # moveit_ros_planning_interface
  # interactive_markers
  # tf2_geometry_msgs
  # moveit_ros_planning
  # pluginlib
  # Eigen3
  # Boost
  # control_msgs
  # )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/String.msg"

  DEPENDENCIES builtin_interfaces  sensor_msgs 
  )
  # "msg/Transform.msg"
  # "msg/Point.msg"
  # "msg/Image.msg"
  # "msg/PointCloud.msg"
  # "msg/PoseArray.msg"
  # "msg/PolyData.msg"
  # "msg/Vector.msg"
  # geometry_msgs

ament_export_dependencies(rosidl_default_runtime)


#include_directories(include ${OpenIGTLink_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/testing/include ${VTK_INCLUDE_DIRS})
include_directories(include ${OpenIGTLink_INCLUDE_DIRS})

add_executable(parameter_event_handler src/parameter_event_handler.cpp)
add_executable(igtl_node  src/main.cpp  src/igtl_node.cpp src/rib_converter_manager.cpp
src/rib_converter_base.cpp)

ament_target_dependencies(parameter_event_handler rclcpp)
ament_target_dependencies(igtl_node rclcpp std_msgs sensor_msgs OpenIGTLink) # sensor_msgs  OpenIGTLink

# target_include_directories(igtl_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

rosidl_target_interfaces(igtl_node ${PROJECT_NAME} rosidl_typesupport_cpp)

target_include_directories(igtl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  parameter_event_handler
  igtl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
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
