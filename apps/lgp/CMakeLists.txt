############################################################
# CMakeLists for the logic-opt library.
#
# Copyright 2020. All Rights Reserved.
#
# Created: October 1, 2020
# Authors: Toki Migimatsu
############################################################

set(BIN_NAME lgp)
add_executable(${BIN_NAME} main.cc)

target_compile_features(${BIN_NAME} PUBLIC cxx_std_17)
set_target_properties(${LIB_NAME} PROPERTIES CXX_EXTENSIONS OFF)

if(LOGIC_OPT_CLANG_TIDY)
    target_enable_clang_tidy(${BIN_NAME})
endif()

target_link_libraries(${BIN_NAME}
  PRIVATE
    logic_opt::logic_opt
)

