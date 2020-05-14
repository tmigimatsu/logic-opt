############################################################
# CMakeLists for Pybind11
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 7, 2018
# Authors: Toki Migimatsu
############################################################

#############
# Pybind11 #
#############

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/logicopt)

set(PYBIND11_LIB_DIR ${ctrl_utils_EXTERNAL_DIR}/pybind11.git)
add_subdirectory(${PYBIND11_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pybind11)

pybind11_add_module(logicopt validator.cc ${LOGIC_OPT_PLANNING_SRC})

target_include_directories(logicopt
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
)
target_link_libraries(logicopt PUBLIC pybind11::pybind11 ${VAL_LIB})
