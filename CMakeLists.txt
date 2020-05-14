############################################################
# CMakeLists for the SpatialDyn library
#
# Copyright 2018. All Rights Reserved.
#
# Created: May 7, 2018
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

# Define project
project(logic_opt VERSION 1.0.0 LANGUAGES CXX)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
option(BUILD_OPTIMIZER "Build logic-opt" ON)

# Define directories
set(LOGIC_OPT_LIB logic_opt)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(LIB_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(LIB_EXTERNAL_DIR ${PROJECT_SOURCE_DIR}/external)
set(LIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(LIB_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# # Create LogicOpt library
# add_library(${LOGIC_OPT_LIB} SHARED)

# # Create namespaced alias for use inside the build tree
# add_library(${LOGIC_OPT_LIB}::${LOGIC_OPT_LIB} ALIAS ${LOGIC_OPT_LIB})

# # Set compiler options
# target_compile_options(${LOGIC_OPT_LIB} PRIVATE
#     $<$<CXX_COMPILER_ID:GNU>:-Wall>
#     $<$<CONFIG:RELEASE>:-O3>
# )

# Find dependencies
list(INSERT CMAKE_MODULE_PATH 0 ${LIB_CMAKE_DIR})

configure_file(${LIB_CMAKE_DIR}/FindIpopt.cmake.in
    ${CMAKE_BINARY_DIR}/FindIpopt.cmake @ONLY)

function(download_package PACKAGE_NAME)
    set(EXT_PACKAGE_VERSION ${ARGV1})
    if(EXISTS ${LIB_CMAKE_DIR}/Find${PACKAGE_NAME}.cmake.in)
        configure_file(${LIB_CMAKE_DIR}/Find${PACKAGE_NAME}.cmake.in
            ${CMAKE_BINARY_DIR}/Find${PACKAGE_NAME}.cmake @ONLY)
    endif()
    configure_file(${LIB_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt @ONLY)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

download_package(VAL)

set(VAL_LIB VAL)
set(VAL_LIB_DIR ${CMAKE_BINARY_DIR}/VAL/build)
set(VAL_SRC_DIR ${VAL_LIB_DIR}/src)
set(VAL_INCLUDE_DIR ${VAL_LIB_DIR}/include)
add_library(${VAL_LIB} STATIC)

# Set include directories
target_include_directories(${VAL_LIB}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${VAL_INCLUDE_DIR}>
)

# Attach sources
target_sources(${VAL_LIB} PRIVATE
    ${VAL_SRC_DIR}/DebugWriteController.cpp
    ${VAL_SRC_DIR}/pddl+.cpp
    ${VAL_SRC_DIR}/ptree.cpp
    ${VAL_SRC_DIR}/typecheck.cpp
)

set(PDDL_BIN pddl)

set(LOGIC_OPT_PLANNING_SRC
    ${LIB_SRC_DIR}/planning/actions.cc
    ${LIB_SRC_DIR}/planning/objects.cc
    ${LIB_SRC_DIR}/planning/parameter_generator.cc
    ${LIB_SRC_DIR}/planning/pddl.cc
    ${LIB_SRC_DIR}/planning/planner.cc
    ${LIB_SRC_DIR}/planning/proposition.cc
    ${LIB_SRC_DIR}/planning/validator.cc
)

add_executable(${PDDL_BIN} ${LIB_SRC_DIR}/pddl.cc ${LOGIC_OPT_PLANNING_SRC})

target_include_directories(${PDDL_BIN}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
)

find_package(ctrl_utils 1.2.0 QUIET)
if(NOT ctrl_utils_FOUND)
    download_package(ctrl_utils 1.2.0)
    find_package(ctrl_utils 1.2.0 REQUIRED)
endif()

target_link_libraries(${PDDL_BIN} PRIVATE
    ${VAL_LIB}
)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_BINARY_DIR})

if(BUILD_OPTIMIZER)

find_package(spatial_dyn 1.2.0 QUIET)
if(NOT spatial_dyn_FOUND)
    download_package(spatial_dyn 1.2.0)
    find_package(spatial_dyn 1.2.0 REQUIRED)
endif()

find_package(redis_gl QUIET)
if(NOT redis_gl_FOUND)
    download_package(redis_gl)
    find_package(redis_gl REQUIRED)
endif()

#find_package(Gurobi 8.1.0 REQUIRED)

find_package(NLopt 2.5.0 QUIET)
if(NOT NLopt_FOUND)
    download_package(NLopt)
    find_package(NLopt 2.5.0 REQUIRED PATHS ${CMAKE_BINARY_DIR}/NLopt/build)
endif()

find_package(Ipopt 3.12.12 QUIET)
if(NOT Ipopt_FOUND)
    download_package(Ipopt)
    find_package(Ipopt 3.12.12 REQUIRED)
endif()

find_package(ncollide_cpp 0.1.0 QUIET)
if(NOT ncollide_cpp_FOUND)
    download_package(ncollide_cpp 0.1.0)
    find_package(ncollide_cpp 0.1.0 REQUIRED)
endif()

find_package(Matlab QUIET)

set(LOGIC_OPT_SRC
    ${LIB_SRC_DIR}/constraints/cartesian_pose_constraint.cc
    ${LIB_SRC_DIR}/constraints/collision_constraint.cc
    ${LIB_SRC_DIR}/constraints/multi_constraint.cc
    ${LIB_SRC_DIR}/constraints/pick_constraint.cc
    ${LIB_SRC_DIR}/constraints/place_constraint.cc
    ${LIB_SRC_DIR}/constraints/push_constraint.cc
    ${LIB_SRC_DIR}/constraints/throw_constraint.cc
    ${LIB_SRC_DIR}/constraints/touch_constraint.cc
    ${LIB_SRC_DIR}/constraints/trajectory_constraint.cc
    ${LIB_SRC_DIR}/constraints/workspace_constraint.cc
    ${LIB_SRC_DIR}/optimization/ipopt.cc
    ${LIB_SRC_DIR}/optimization/nlopt.cc
    ${LIB_SRC_DIR}/optimization/objectives.cc
    ${LIB_SRC_DIR}/world.cc
)

set(TRAJ_BIN traj)
add_executable(${TRAJ_BIN}
               ${LIB_SRC_DIR}/traj.cc
               ${LIB_SRC_DIR}/control/opspace_controller.cc
               ${LOGIC_OPT_SRC})
if(Matlab_FOUND)
    target_sources(${TRAJ_BIN} ${LIB_SRC_DIR}/matlab/throw_constraint_scp.cc)
endif()

target_link_libraries(${TRAJ_BIN} PRIVATE
    spatial_dyn::spatial_dyn
    ctrl_utils::ctrl_utils
    redis_gl::redis_gl
    ncollide_cpp::ncollide_cpp
    NLopt::nlopt
    Ipopt::Ipopt
)

if(Matlab_FOUND)
    target_link_libraries(${TRAJ_BIN} PRIVATE Matlab::Matlab)
endif()

target_include_directories(${TRAJ_BIN}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
)

set(LGP_BIN lgp)
add_executable(${LGP_BIN}
               ${LIB_SRC_DIR}/main.cc
               ${LIB_SRC_DIR}/control/opspace_controller.cc
               ${LOGIC_OPT_SRC}
               ${LOGIC_OPT_PLANNING_SRC})
if(Matlab_FOUND)
    target_sources(${BIN_BIN} ${LIB_SRC_DIR}/matlab/throw_constraint_scp.cc)
endif()

target_link_libraries(${LGP_BIN} PRIVATE
    spatial_dyn::spatial_dyn
    ctrl_utils::ctrl_utils
    redis_gl::redis_gl
    ncollide_cpp::ncollide_cpp
    NLopt::nlopt
    Ipopt::Ipopt
    ${VAL_LIB}
)
if(Matlab_FOUND)
    target_link_libraries(${TRAJ_BIN} PRIVATE Matlab::Matlab)
endif()

target_include_directories(${LGP_BIN}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
)

endif(BUILD_OPTIMIZER)

# configure_file(${CMAKE_BINARY_DIR}/VAL/build/parser ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)
# configure_file(${CMAKE_BINARY_DIR}/VAL/build/validate ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COPYONLY)

# Use GNUInstalDirs to install ibraries into correct locations on all platforms
include(GNUInstallDirs)

# set(SPATIAL_DYN_TARGETS ${SPATIAL_DYN_LIB}Targets)
# set(SPATIAL_DYN_CONFIG ${SPATIAL_DYN_LIB}Config)
# set(SPATIAL_DYN_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${SPATIAL_DYN_LIB})

# # Install the library
# install(TARGETS ${SPATIAL_DYN_LIB}
#     EXPORT ${SPATIAL_DYN_TARGETS}
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

# # Install the headers
# install(DIRECTORY ${SPATIAL_DYN_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# # Export targets to Targets.cmake
# install(EXPORT ${SPATIAL_DYN_TARGETS}
#     DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR}
#     NAMESPACE ${SPATIAL_DYN_LIB}::
#     FILE ${SPATIAL_DYN_TARGETS}.cmake
# )

# # Create ConfigVersion.cmake
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}Version.cmake
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
# )

# # Create Config.cmake
# configure_package_config_file(${SPATIAL_DYN_CMAKE_DIR}/${SPATIAL_DYN_CONFIG}.cmake.in
#     ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
#     INSTALL_DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR})

# # Install config script files
# install(FILES
#     ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}.cmake
#     ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_CONFIG}Version.cmake
#     DESTINATION ${SPATIAL_DYN_CONFIG_INSTALL_DIR}
# )

# Build tests
# include(CTest)
if(BUILD_TESTING)
    add_subdirectory(test)
    if(Matlab_FOUND)
        add_subdirectory(src/matlab)
    endif()
endif()

# Build python wrapper
add_subdirectory(${LIB_SRC_DIR}/python)

# # Export for build tree
# export(TARGETS ${SPATIAL_DYN_LIB_TARGETS}
#     NAMESPACE ${SPATIAL_DYN_LIB}::
#     FILE ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_TARGETS}.cmake)

# # Register package in user registry
# export(PACKAGE ${SPATIAL_DYN_LIB})

