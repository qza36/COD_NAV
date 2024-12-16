cmake_minimum_required(VERSION 3.8)
project(clear_costmap_caller)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)


add_executable(clear_costmap_caller src/clear_costmap_caller.cpp)


ament_target_dependencies(clear_costmap_caller
        rclcpp
        nav2_msgs
)


install(TARGETS clear_costmap_caller
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
