cmake_minimum_required(VERSION 3.5)

project(gmsl_ros2)

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

# Gstreamer doesn't provide CMake files
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# GMSL node
#=============

set(GMSL_NODE_SOURCES
  src/gscam_node.cpp)

set(GMSL_NODE_DEPS
  camera_calibration_parsers
  camera_info_manager
  class_loader
  GST_APP
  rclcpp
  rclcpp_components
  sensor_msgs)

add_library(gmsl_node SHARED
  ${GMSL_NODE_SOURCES})

target_compile_definitions(gmsl_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(gmsl_node
  ${GMSL_NODE_DEPS})

rclcpp_components_register_nodes(gmsl_node "gscam2::GSCamNode")
set(node_plugins "${node_plugins}gscam2::GSCamNode;$<TARGET_FILE:gmsl_node>\n")

#=============
# Test subscriber node
#=============

set(SUBSCRIBER_NODE_SOURCES
  src/subscriber_node.cpp)

set(SUBSCRIBER_NODE_DEPS
  class_loader
  rclcpp
  rclcpp_components
  sensor_msgs)

add_library(subscriber_node SHARED
  ${SUBSCRIBER_NODE_SOURCES})

target_compile_definitions(subscriber_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(subscriber_node
  ${SUBSCRIBER_NODE_DEPS})

rclcpp_components_register_nodes(subscriber_node "gscam2::ImageSubscriberNode")
set(node_plugins "${node_plugins}gscam2::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

#=============
# GMSL main, statically linked to make IDE debugging easier
#=============

add_executable(gmsl_main
  src/gscam_main.cpp
  ${GMSL_NODE_SOURCES})

ament_target_dependencies(gmsl_main
  ${GMSL_NODE_DEPS})

#=============
# Manual composition of camera and subscriber nodes, IPC=true
#=============

# add_executable(
#   ipc_test_main
#   src/ipc_test_main.cpp
# )
# target_link_libraries(
#   ipc_test_main
#   gmsl_node
#   subscriber_node
# )
# ament_target_dependencies(
#   ipc_test_main
#   rclcpp
# )

#=============
# Test
#=============

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(
    smoke_test
    test/smoke_test.cpp
    ENV GST_CONFIG="v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=800,height=600 ! videoconvert"
  )
  if(TARGET smoke_test)
    target_link_libraries(smoke_test gmsl_node)
  endif()
endif()

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_gmsl_node export_subscriber_node)

ament_export_libraries(gmsl_node subscriber_node)

ament_package()

#=============
# Install
#=============

install(DIRECTORY include/gscam2 DESTINATION include)

install(
  TARGETS gmsl_node
  EXPORT export_gmsl_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS subscriber_node
  EXPORT export_subscriber_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  gmsl_main
  # ipc_test_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg launch rviz DESTINATION share/${PROJECT_NAME})