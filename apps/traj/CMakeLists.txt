############################################################
# CMakeLists for the symbolic library.
#
# Copyright 2020. All Rights Reserved.
#
# Created: October 1, 2020
# Authors: Toki Migimatsu
############################################################

set(BIN_NAME traj)
add_executable(${BIN_NAME} main.cc)

target_compile_features(${BIN_NAME} PUBLIC cxx_std_17)

lib_add_subdirectory(ctrl_utils)
ctrl_utils_add_subdirectory(cpp_redis)
lib_add_subdirectory(redis_gl)
target_link_libraries(${BIN_NAME}
  PRIVATE
    logic_opt::logic_opt
    cpp_redis::cpp_redis
    redis_gl::redis_gl
)

if(SYMBOLIC_CLANG_TIDY)
    target_enable_clang_tidy(${BIN_NAME})
endif()
