cmake_minimum_required(VERSION 3.5)

project(scene_manipulation_bringup)

find_package(ament_cmake REQUIRED)

# if using option 1, the scenatio folder has to be copied to the share folder
install(DIRECTORY launch config scenario_1
 DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
