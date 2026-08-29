set(CMAKE_INCLUDE_CURRENT_DIR ON)

# LINKS the public libraries rather than splicing $<TARGET_OBJECTS:...>, for the same reason
# examples/query_async does: CXX_VISIBILITY_PRESET=hidden means only the dew_* C entry points are in
# the shared library, so an ordinary consumer cannot accidentally reach internal C++. That is
# exactly the property being tested here -- this importer is the proof that the public surface is
# complete enough to drive from outside, which is what the Python half will then do.
add_executable(ford_import
        ford_import.cpp
        ford_colour.cpp
        ford_convert.cpp
        mat_v5.cpp
        ppm.cpp
        ford_dataset.cpp
)

# zlib-ng in native mode: <zlib-ng.h>, zng_* symbols. See CMake/Build3rdParty.cmake.
target_link_libraries(ford_import PRIVATE dew_converter dew_access dew_core)
target_link_libraries(ford_import PRIVATE zlib-ng fmt)
# dew_render too, even though nothing here draws anything: dew_converter carries the renderer's
# data source (data_source_converter) and so links against it.
copy_dll_for_target(ford_import dew_converter dew_access dew_core dew_render)
setWarningFlagsForTarget(ford_import)
set_example_properties(ford_import)
