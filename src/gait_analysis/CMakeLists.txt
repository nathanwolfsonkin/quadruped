cmake_minimum_required(VERSION 3.8)
project(gait_analysis)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cross_correlation_node src/cross_correlation_node.cpp)
ament_target_dependencies(cross_correlation_node rclcpp std_msgs)

target_include_directories(cross_correlation_node PRIVATE include/gait_analysis)

install(TARGETS cross_correlation_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
