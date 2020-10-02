############################################################
# CMakeLists for redis_gl
#
# Copyright 2020. All Rights Reserved.
#
# Created: September 23, 2018
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET redis_gl::redis_gl)
    init_git_submodule(redis-gl.git)
    add_subdirectory(redis-gl.git)
endif()
