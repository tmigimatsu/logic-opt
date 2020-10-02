############################################################
# CMakeLists for the logic-opt library.
#
# Copyright 2018. All Rights Reserved.
#
# Created: May 7, 2018
# Authors: Toki Migimatsu
############################################################

cmake_minimum_required(VERSION 3.11)

# Define project.
project(logic_opt
    VERSION 1.1.0
    DESCRIPTION "Object-Centric Task and Motion Planning"
    LANGUAGES CXX)

# Detect whether this project is built by itself or included as a subdirectory.
include(CMakeDependentOption)
cmake_dependent_option(LOGIC_OPT_MAIN_BUILD "Build symbolic as a main project."
    ON "CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME" OFF)

# Define main build options.
if(LOGIC_OPT_MAIN_BUILD)
    option(CLANG_TIDY "Perform clang-tidy checks." ON)
endif()

# Define CMake options.
cmake_dependent_option(LOGIC_OPT_CLANG_TIDY "Perform clang-tidy checks." ON "LOGIC_OPT_MAIN_BUILD AND CLANG_TIDY" OFF)

# Set default build type to release.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set the output directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Set output directories.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Include custom CMake utilities.
include(cmake/utils.cmake)

# Build the library.
add_subdirectory(src)

# Build examples.
add_subdirectory(apps)

# # Set CMake flags
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
#     set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
# endif()
# option(BUILD_OPTIMIZER "Build logic-opt" ON)

# # Define directories
# set(LOGIC_OPT_LIB logic_opt)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(LIB_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
# set(LIB_EXTERNAL_DIR ${PROJECT_SOURCE_DIR}/external)
# set(LIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
# set(LIB_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# # Find dependencies
# include(${LIB_CMAKE_DIR}/include_package.cmake)

# add_subdirectory(${LIB_EXTERNAL_DIR}/ctrl_utils)
# ctrl_utils_add_subdirectory(cpp_redis)
# add_subdirectory(${LIB_EXTERNAL_DIR}/symbolic)
# add_subdirectory(${LIB_EXTERNAL_DIR}/spatial_opt)
# add_subdirectory(${LIB_EXTERNAL_DIR}/redis_gl)

# set(LOGIC_OPT_SRC
#     ${LIB_SRC_DIR}/constraints/cartesian_pose_constraint.cc
#     ${LIB_SRC_DIR}/constraints/collision_constraint.cc
#     ${LIB_SRC_DIR}/constraints/multi_constraint.cc
#     ${LIB_SRC_DIR}/constraints/pick_constraint.cc
#     ${LIB_SRC_DIR}/constraints/place_constraint.cc
#     ${LIB_SRC_DIR}/constraints/push_constraint.cc
#     ${LIB_SRC_DIR}/constraints/throw_constraint.cc
#     ${LIB_SRC_DIR}/constraints/touch_constraint.cc
#     ${LIB_SRC_DIR}/constraints/trajectory_constraint.cc
#     ${LIB_SRC_DIR}/constraints/workspace_constraint.cc
#     ${LIB_SRC_DIR}/optimization/ipopt.cc
#     ${LIB_SRC_DIR}/optimization/nlopt.cc
#     ${LIB_SRC_DIR}/optimization/objectives.cc
#     ${LIB_SRC_DIR}/world.cc
# )

# set(LGP_BIN lgp)
# add_executable(${LGP_BIN}
#     ${LIB_SRC_DIR}/main.cc
#     ${LIB_SRC_DIR}/control/opspace_controller.cc
#     ${LOGIC_OPT_SRC}
#     ${LOGIC_OPT_PLANNING_SRC}
# )

# target_link_libraries(${LGP_BIN} PRIVATE
#     ctrl_utils::ctrl_utils
#     spatial_opt::spatial_opt
#     cpp_redis::cpp_redis
#     redis_gl::redis_gl
# )

# target_include_directories(${LGP_BIN} PRIVATE
#     ${LIB_INCLUDE_DIR}
# )

# # Build tests
# include(CTest)
# if(BUILD_TESTING)
#     add_subdirectory(test)
# endif()

# # Build python wrapper
# add_subdirectory(${LIB_SRC_DIR}/python)

# # Build opspace controller
# add_subdirectory(${LIB_SRC_DIR}/opspace)
