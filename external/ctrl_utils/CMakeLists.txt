############################################################
# CMakeLists for ctrl_utils
#
# Copyright 2020. All Rights Reserved.
#
# Created: September 23, 2020
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET ctrl_utils::ctrl_utils)
    init_git_submodule(ctrl-utils.git)
    add_subdirectory(ctrl-utils.git)
endif()
