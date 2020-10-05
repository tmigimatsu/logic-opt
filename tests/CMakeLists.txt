############################################################
# CMakeLists for the spatial_opt tests.
#
# Copyright 2020. All Rights Reserved.
#
# Created: May 25, 2020
# Authors: Toki Migimatsu
############################################################

message(STATUS "Configuring ${PROJECT_NAME} tests.")

ctrl_utils_add_subdirectory(Catch2)

set(TESTS_BIN tests)
add_executable(${TESTS_BIN}
               main.cc
               jacobian_tests.cc)

target_link_libraries(${TESTS_BIN} PRIVATE
    logic_opt::logic_opt
    Catch2::Catch2
)
