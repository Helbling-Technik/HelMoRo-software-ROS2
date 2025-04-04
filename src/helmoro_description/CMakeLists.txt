cmake_minimum_required(VERSION 3.8)
project(helmoro_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

install(
  DIRECTORY launch meshes test urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/launch_description.test.py)
endif()

ament_package()
