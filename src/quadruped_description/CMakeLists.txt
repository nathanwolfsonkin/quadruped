cmake_minimum_required(VERSION 3.5)

project(quadruped_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install models and Xacro files
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY xacro/
  DESTINATION share/${PROJECT_NAME}/xacro
)

# Set paths to Xacro and SDF files
set(XACRO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/xacro/model.xacro)
set(SDF_FILE ${CMAKE_CURRENT_BINARY_DIR}/models/quadruped/model.sdf)

# Custom command to generate SDF from Xacro
add_custom_command(
  OUTPUT ${SDF_FILE}
  COMMAND xacro ${XACRO_FILE} -o ${SDF_FILE}
  DEPENDS ${XACRO_FILE}
  COMMENT "Generating SDF from Xacro"
)

# Custom target to build SDF
add_custom_target(generate_sdf ALL DEPENDS ${SDF_FILE})

# Install the generated SDF
install(
  FILES ${SDF_FILE}
  DESTINATION share/${PROJECT_NAME}/models/quadruped
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
