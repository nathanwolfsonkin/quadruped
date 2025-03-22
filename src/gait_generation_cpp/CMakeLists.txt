cmake_minimum_required(VERSION 3.8)
project(gait_generation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# Ensure headers are found
include_directories(include)

# Add executable
add_executable(analytical_gait_loader src/analytical_gait_loader.cpp)

# Link against required libraries
target_include_directories(analytical_gait_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(analytical_gait_loader yaml-cpp)
ament_target_dependencies(analytical_gait_loader rclcpp yaml-cpp std_msgs rosgraph_msgs)

# Install
install(TARGETS analytical_gait_loader
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/)

install(TARGETS analytical_gait_loader
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
