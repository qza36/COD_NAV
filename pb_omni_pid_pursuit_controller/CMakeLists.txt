cmake_minimum_required(VERSION 3.10)
project(pb_omni_pid_pursuit_controller)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp
  src/pid.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(nav2_core pb_omni_pid_pursuit_controller.xml)

ament_auto_package()
