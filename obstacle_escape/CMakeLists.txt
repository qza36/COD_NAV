cmake_minimum_required(VERSION 3.8)
project(obstacle_escape)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include C++ directories
include_directories(include)

# Add C++ executable
add_executable(obstacle_escape_node src/obstacle_escape_node.cpp)
ament_target_dependencies(obstacle_escape_node
        rclcpp
        nav_msgs
        geometry_msgs
)

# Install C++ executables
install(TARGETS
        obstacle_escape_node
        DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()