cmake_minimum_required(VERSION 3.8)
project(gnss_to_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  bondcpp
  std_msgs
  nav_msgs
  gps_msgs
  mavros_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  GeographicLib
  angles
)

find_package(ament_cmake REQUIRED)
# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/gnss_to_odom.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::GnssToOdom"
  EXECUTABLE ${PROJECT_NAME}_node)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
