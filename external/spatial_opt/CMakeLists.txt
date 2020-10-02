############################################################
# CMakeLists for spatial_opt
#
# Copyright 2020. All Rights Reserved.
#
# Created: September 30, 2020
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET spatial_opt::spatial_opt)
    init_git_submodule(spatial-opt.git)
    add_subdirectory(spatial-opt.git)
endif()
