cmake_minimum_required(VERSION 3.5)

project(helmoro_motor_commands)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(helmoro_msgs REQUIRED)
find_package(helmoro_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

### Libraries

set(dependencies
  geometry_msgs
  sensor_msgs
  helmoro_msgs
  helmoro_description
  rclcpp
  rclcpp_components
)

# Helmoro Motor Command
add_library(helmoro_motor_commands_lib SHARED)
target_sources(
  helmoro_motor_commands_lib
  PRIVATE
    src/helmoro_motor_commands.cpp
)
target_include_directories(helmoro_motor_commands_lib PUBLIC include)
ament_target_dependencies(helmoro_motor_commands_lib
  ${dependencies}
)

rclcpp_components_register_node(helmoro_motor_commands_lib
  PLUGIN "helmoro_motor_commands::HelmoroMotorCommands"
  EXECUTABLE helmoro_motor_commands)

# Library names
  set(libraries_names
  helmoro_motor_commands_lib
)
 
### Install

install(TARGETS ${libraries_names}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    param
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${libraries_names})
ament_export_dependencies(${dependencies})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
