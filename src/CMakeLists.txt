############################################################
# CMakeLists for the logic-opt library.
#
# Copyright 2020. All Rights Reserved.
#
# Created: May 7, 2020
# Authors: Toki Migimatsu
############################################################

set(LIB_NAME ${PROJECT_NAME})

# Create the library.
add_library(${LIB_NAME} SHARED)
add_library(${LIB_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

target_compile_features(${LIB_NAME} PUBLIC cxx_std_17)
set_target_properties(${LIB_NAME} PROPERTIES CXX_EXTENSIONS OFF)

if(LOGIC_OPT_CLANG_TIDY)
    target_enable_clang_tidy(${LIB_NAME})
endif()

target_sources(${LIB_NAME}
  PRIVATE
    control/opspace_controller.cc
    constraints/cartesian_pose_constraint.cc
    constraints/collision_constraint.cc
    constraints/pick_constraint.cc
    constraints/place_constraint.cc
    constraints/push_constraint.cc
    # constraints/surface_contact_constraint.cc
    constraints/throw_constraint.cc
    constraints/touch_constraint.cc
    constraints/trajectory_constraint.cc
    constraints/workspace_constraint.cc
    objectives/angular_distance_objective.cc
    objectives/linear_distance_objective.cc
    objectives/workspace_objective.cc
    object.cc
    world.cc
)

set(LIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
)

lib_add_subdirectory(ctrl_utils)
ctrl_utils_add_subdirectory(cpp_redis)
lib_add_subdirectory(spatial_opt INCLUDE_ALL_TARGETS)
lib_add_subdirectory(symbolic)
lib_add_subdirectory(redis_gl)
target_link_libraries(${LIB_NAME}
  PUBLIC
    spatial_opt::spatial_opt
    symbolic::symbolic
    redis_gl::redis_gl
    cpp_redis::cpp_redis
)

# Install the library.
install(TARGETS ${LIB_NAME}
    EXPORT ${LIB_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the headers.
install(DIRECTORY ${LIB_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Create LibConfig.cmake.
set(LIB_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${LIB_NAME})
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/${LIB_NAME}Config.cmake.in
    ${PROJECT_BINARY_DIR}/${LIB_NAME}Config.cmake
    INSTALL_DESTINATION ${LIB_CONFIG_INSTALL_DIR}
)

# Create LibConfigVersion.cmake.
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Install LibConfig.cmake and LibConfigVersion.cmake.
install(FILES
    ${PROJECT_BINARY_DIR}/${LIB_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${LIB_NAME}ConfigVersion.cmake
    DESTINATION ${LIB_CONFIG_INSTALL_DIR}
)

# Export targets to LibTargets.cmake.
install(EXPORT ${LIB_NAME}Targets
    DESTINATION ${LIB_CONFIG_INSTALL_DIR}
    NAMESPACE ${LIB_NAME}::
)

# Export targets to LibTargets.cmake for the build tree.
export(TARGETS ${LIB_NAME}
    NAMESPACE ${LIB_NAME}::
    FILE ${LIB_NAME}Targets.cmake
)

