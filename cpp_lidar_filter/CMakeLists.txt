cmake_minimum_required(VERSION 3.8)
project(cpp_lidar_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(lidar_filter_node src/filter_node.cpp)
ament_target_dependencies(lidar_filter_node
        rclcpp sensor_msgs
        pcl_conversions
        pcl_ros
        visualization_msgs
)

target_link_libraries(lidar_filter_node ${PCL_LIBRARIES})
target_include_directories(lidar_filter_node PUBLIC ${PCL_INCLUDE_DIRS})

install(TARGETS
        lidar_filter_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
