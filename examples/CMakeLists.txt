function(set_example_properties target)
    if (APPLE)
        set_target_properties(${target} PROPERTIES
                BUILD_RPATH "@executable_path/../lib"
        )
    endif ()
endfunction()

add_subdirectory(renderer)
add_subdirectory(query_async)
add_subdirectory(ford)
