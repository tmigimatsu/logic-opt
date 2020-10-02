############################################################
# Utility scripts for downloading external packages
#
# Copyright 2020. All Rights Reserved.
#
# Created: June 5, 2020
# Authors: Toki Migimatsu
############################################################

function(init_git_submodule GIT_SUBMODULE)
    set(RECURSIVE "")
    if(DEFINED ARGV1)
        if (${ARGV1})
            set(RECURSIVE "--recursive")
        endif()
    endif()

    # Update submodule
    find_package(Git REQUIRED)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" submodule update --init ${RECURSIVE} ${GIT_SUBMODULE}
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        RESULT_VARIABLE git_submodule_result
    )
    if(NOT git_submodule_result EQUAL "0")
        message(FATAL_ERROR "${GIT_EXECUTABLE} submodule update --init ${RECURSIVE} ${GIT_SUBMODULE} failed with error:\n ${git_submodule_result}")
    endif()
endfunction()

function(lib_add_subdirectory SUBDIRECTORY)
    set(LIB_EXTERNAL_DIR "${PROJECT_SOURCE_DIR}/external")
    set(EXTERNAL_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/external")
    add_subdirectory("${LIB_EXTERNAL_DIR}/${SUBDIRECTORY}"
        "${EXTERNAL_BINARY_DIR}/${SUBDIRECTORY}"
        EXCLUDE_FROM_ALL
    )
endfunction()

function(target_enable_clang_tidy MYTARGET)
    find_program(CLANG_TIDY_EXECUTABLE NAMES "clang-tidy")
    if(CLANG_TIDY_EXECUTABLE STREQUAL "CLANG_TIDY_EXECUTABLE-NOTFOUND")
        message(WARNING "Unable to find clang-tidy for ${MYTARGET}.")
    else()
        message(STATUS "Enabling clang-tidy for ${MYTARGET}.")
        set_target_properties(${MYTARGET} PROPERTIES CXX_CLANG_TIDY "${CLANG_TIDY_EXECUTABLE}")
    endif()
endfunction()
