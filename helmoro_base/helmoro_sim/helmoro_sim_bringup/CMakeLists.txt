cmake_minimum_required(VERSION 3.8)

project(helmoro_sim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros_ign_interfaces QUIET)

if(NOT ros_ign_interfaces_FOUND)
  message(WARNING "ros_ign_interfaces not found. Skipping ${PROJECT_NAME} package.")
  ament_package()
  return()
endif()

install(
  DIRECTORY 
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()