cmake_minimum_required(VERSION 3.5)

project(helmoro_gazebo_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(helmoro_msgs REQUIRED)
find_package(helmoro_description REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ros_gz QUIET)
if(NOT ros_gz_FOUND)
  message(WARNING "Gazebo not found. Skipping ${PROJECT_NAME} package.")
  ament_package()
  return()
endif()

# Add libraries
add_library(helmoro_gazebo_plugin SHARED
  src/helmoro_gazebo_plugin.cpp
)
target_include_directories(helmoro_gazebo_plugin PUBLIC include)
ament_target_dependencies(helmoro_gazebo_plugin
  "gazebo_dev"
  "gazebo_ros"
  "helmoro_msgs"
  "helmoro_description"
  "sensor_msgs"
  "rclcpp"
)

ament_export_libraries(helmoro_gazebo_plugin)

# Exports
ament_export_include_directories(include)
ament_export_dependencies(ros_gz)
ament_export_dependencies(helmoro_msgs)
ament_export_dependencies(rclcpp)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
    helmoro_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()