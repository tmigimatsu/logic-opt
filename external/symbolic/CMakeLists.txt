############################################################
# CMakeLists for symbolic
#
# Copyright 2020. All Rights Reserved.
#
# Created: September 30, 2020
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET symbolic::symbolic)
    init_git_submodule(symbolic.git)
    add_subdirectory(symbolic.git)
endif()
