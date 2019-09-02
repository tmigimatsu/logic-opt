############################################################
# Finds the ccd library
#
# Copyright 2019. All Rights Reserved.
#
# Created: June 4, 2019
# Authors: Toki Migimatsu
############################################################

list(REMOVE_AT CMAKE_MODULE_PATH 0)
find_package(Matlab)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_BINARY_DIR})

if(NOT TARGET Matlab::Matlab)
    add_library(Matlab::Matlab INTERFACE IMPORTED)
    set(Matlab_LIBRARIES
        ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabEngine.so
        ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabDataArray.so
    )
    set_target_properties(Matlab::Matlab PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Matlab_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${Matlab_LIBRARIES}"
    )
endif()
