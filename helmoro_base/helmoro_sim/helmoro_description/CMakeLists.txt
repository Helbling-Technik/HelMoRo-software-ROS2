cmake_minimum_required(VERSION 3.5)

project(helmoro_description)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY
    meshes
    launch
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()