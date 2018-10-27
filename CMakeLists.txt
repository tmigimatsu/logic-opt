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
project(traj_opt VERSION 1.0.0 LANGUAGES CXX)

# Set CMake flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Define directories
set(TRAJ_OPT_LIB TrajOpt)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(TRAJ_OPT_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(TRAJ_OPT_EXT_DIR ${PROJECT_SOURCE_DIR}/external)
set(TRAJ_OPT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(TRAJ_OPT_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# # Create TrajOpt library
# add_library(${TRAJ_OPT_LIB} SHARED)

# # Create namespaced alias for use inside the build tree
# add_library(${TRAJ_OPT_LIB}::${TRAJ_OPT_LIB} ALIAS ${TRAJ_OPT_LIB})

# # Set compiler options
# target_compile_options(${TRAJ_OPT_LIB} PRIVATE
#     $<$<CXX_COMPILER_ID:GNU>:-Wall>
#     $<$<CONFIG:RELEASE>:-O3>
# )

# Find dependencies
list(INSERT CMAKE_MODULE_PATH 0 ${TRAJ_OPT_CMAKE_DIR})

configure_file(${TRAJ_OPT_CMAKE_DIR}/FindIpopt.cmake.in
    ${CMAKE_BINARY_DIR}/FindIpopt.cmake @ONLY)

function(download_package PACKAGE_NAME)
    configure_file(${TRAJ_OPT_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

find_package(SpatialDyn 1.0.0 REQUIRED)
find_package(Gurobi 8.1.0 REQUIRED)

find_package(NLopt 2.5.0 QUIET)
if(NOT NLopt_FOUND)
    download_package(NLopt)
    find_package(NLopt 2.5.0 REQUIRED PATHS ${CMAKE_BINARY_DIR}/NLopt/build)
endif()

find_package(Ipopt 3.12.11 QUIET)
if(NOT Ipopt_FOUND)
    download_package(Ipopt)
    list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_BINARY_DIR})
    find_package(Ipopt 3.12.11 REQUIRED)
    list(REMOVE_AT CMAKE_MODULE_PATH 0)
endif()

add_executable(main
    ${CMAKE_CURRENT_SOURCE_DIR}/src/constraints.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ipopt.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/objectives.cc
)

target_link_libraries(main PRIVATE
    SpatialDyn::SpatialDyn
    Gurobi::Gurobi
    NLopt::nlopt
    Ipopt::Ipopt
)

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

# # Build tests
# include(CTest)
# if(BUILD_TESTING)
#     add_subdirectory(tests)
# endif()

# # Build python wrapper
# add_subdirectory(${SPATIAL_DYN_SRC_DIR}/python)

# # Export for build tree
# export(TARGETS ${SPATIAL_DYN_LIB_TARGETS}
#     NAMESPACE ${SPATIAL_DYN_LIB}::
#     FILE ${CMAKE_BINARY_DIR}/${SPATIAL_DYN_TARGETS}.cmake)

# # Register package in user registry
# export(PACKAGE ${SPATIAL_DYN_LIB})

