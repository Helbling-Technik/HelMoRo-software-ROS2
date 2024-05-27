cmake_minimum_required(VERSION 3.5)

project(helmoro_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gazebo_ros QUIET)
if(NOT gazebo_ros_FOUND)
  message(WARNING "Gazebo not found. Skipping ${PROJECT_NAME} package.")
  ament_package()
  return()
endif()


install(
  DIRECTORY
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()