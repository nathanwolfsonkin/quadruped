cmake_minimum_required(VERSION 3.5)
project(quadruped_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------
# YAML Generation
# --------------------------
set(PARAMS_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/config/params.yaml)
set(PARAMS_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/config)
set(PARAMS_BUILD ${PARAMS_BUILD_DIR}/params.yaml)
set(YAML_FILL ${CMAKE_CURRENT_SOURCE_DIR}/build/yaml_fill.py)

file(MAKE_DIRECTORY ${PARAMS_BUILD_DIR})

add_custom_command(
  OUTPUT ${PARAMS_BUILD}
  COMMAND python3 ${YAML_FILL} ${PARAMS_SOURCE} ${PARAMS_BUILD}
  DEPENDS ${YAML_FILL} ${PARAMS_SOURCE}
  COMMENT "Generating inertial values for uniform mass distribution"
)

add_custom_target(generate_yaml ALL DEPENDS ${PARAMS_BUILD})

install(
  FILES ${PARAMS_BUILD}
  DESTINATION share/${PROJECT_NAME}/config
)

# --------------------------
# Xacro to URDF/SDF Generation
# --------------------------
set(XACRO_QUAD ${CMAKE_CURRENT_SOURCE_DIR}/xacro/quadruped.xacro)
set(URDF_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/urdf)
set(URDF_QUAD ${URDF_OUTPUT_DIR}/quadruped.urdf)

set(XACRO_QUAD_PLUGINS ${CMAKE_CURRENT_SOURCE_DIR}/xacro/quad_plugins.xacro)
set(SDF_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/models/quadruped)
set(SDF_QUAD_PLUGINS ${SDF_OUTPUT_DIR}/model.sdf)

file(MAKE_DIRECTORY ${URDF_OUTPUT_DIR})
file(MAKE_DIRECTORY ${SDF_OUTPUT_DIR})

file(GLOB_RECURSE XACRO_DEPENDENCIES ${CMAKE_CURRENT_SOURCE_DIR}/xacro/*.xacro)

# Generate URDF from Xacro
add_custom_command(
  OUTPUT ${URDF_QUAD}
  COMMAND xacro ${XACRO_QUAD} -o ${URDF_QUAD}
  DEPENDS ${XACRO_DEPENDENCIES} ${PARAMS_BUILD} generate_yaml
  COMMENT "Generating URDF from Xacro"
)

# Generate SDF from Xacro
add_custom_command(
  OUTPUT ${SDF_QUAD_PLUGINS}
  COMMAND xacro ${XACRO_QUAD_PLUGINS} -o ${SDF_QUAD_PLUGINS}
  DEPENDS ${XACRO_DEPENDENCIES} ${PARAMS_BUILD} generate_yaml
  COMMENT "Generating SDF from Xacro"
)

add_custom_target(generate_urdf ALL DEPENDS ${URDF_QUAD})
add_custom_target(generate_sdf ALL DEPENDS ${SDF_QUAD_PLUGINS})

# --------------------------
# Install Files
# --------------------------
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/models/quadruped/
  DESTINATION share/${PROJECT_NAME}/models/quadruped
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()