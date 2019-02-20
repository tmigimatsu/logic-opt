if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CARGO_CMD cargo build)
    set(TARGET_DIR "debug")
else ()
    set(CARGO_CMD cargo build --release)
    set(TARGET_DIR "release")
endif ()

set(NCOLLIDE_SO "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}/libncollide_cpp.so")

# TODO: Use add_custom_command
add_custom_target(ncollide ALL
    COMMENT "Compiling ncollide module"
    COMMAND CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} ${CARGO_CMD} 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(ncollide PROPERTIES LOCATION ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR})

add_test(NAME ncollide_test
    COMMAND cargo test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
