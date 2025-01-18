cmake_minimum_required(VERSION 3.5)

project(generic_quadruped_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install model and Xacro files
install(
  DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(
  DIRECTORY xacro/
  DESTINATION share/${PROJECT_NAME}/xacro
)

# Set paths to Xacro and SDF files
set(XACRO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/xacro/quadruped.xacro)
set(URDF_FILE ${CMAKE_CURRENT_BINARY_DIR}/model/quadruped.urdf)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/model)

# Add dependency on all files in the xacro directory
file(GLOB_RECURSE XACRO_DEPENDENCIES ${CMAKE_CURRENT_SOURCE_DIR}/xacro/*.xacro)

# Custom command to generate URDF from Xacro
add_custom_command(
  OUTPUT ${URDF_FILE}
  COMMAND xacro ${XACRO_FILE} -o ${URDF_FILE}
  DEPENDS ${XACRO_DEPENDENCIES}
  COMMENT "Generating URDF from Xacro"
)

# Custom target to build urdf
add_custom_target(generate_urdf ALL DEPENDS ${URDF_FILE})

# Install the generated 
install(
  FILES ${URDF_FILE}
  DESTINATION share/${PROJECT_NAME}/model
)

ament_package()
