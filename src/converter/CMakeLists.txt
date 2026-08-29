
set(public_headers
        dew/converter/export.h
        dew/converter/converter.h
        dew/converter/connection_cli.h
        dew/converter/converter_data_source.h
)

set(private_headers
        processor.hpp
        reader.hpp
        processor.hpp
        converter.hpp
        input_header.hpp
        sorter.hpp
        memcpy_array.hpp
        tree_build.hpp
        point_buffer_splitter.hpp
        pre_init_file_retriever.hpp
        storage_handler.hpp
        tree_handler.hpp
        tree_lod_generator.hpp
        data_source_converter.hpp
        data_source_node_bbox.hpp
        frustum_tree_walker.hpp
        render_node.hpp
        render_pipeline.hpp
        input_data_source_registry.hpp
        native_node_data_loader.hpp
)

set(sources
        converter.cpp
        processor.cpp
        reader.cpp
        input_header.cpp
        laszip_file_convert_callbacks.cpp
        sorter.cpp
        tree_build.cpp
        pre_init_file_retriever.cpp
        storage_handler.cpp
        tree_handler.cpp
        tree_collapse.cpp
        tree_lod_generator.cpp
        data_source_converter.cpp
        data_source_node_bbox.cpp
        frustum_tree_walker.cpp
        render_pipeline.cpp
        virtual_tree.cpp
        native_node_data_loader.cpp
        node_decode.cpp
        upload_handler.cpp
        input_data_source_registry.cpp
        attribute_amend.cpp
)

add_library(dew_converter_objects OBJECT ${public_headers} ${private_headers} ${sources})
# dew_core is consumed for its INCLUDE PATHS ONLY, never linked.
#
# Its objects are spliced into the shared library below (they have to be: the cross-module API is
# internal C++ and the build sets CXX_VISIBILITY_PRESET=hidden, so those symbols are not exported from
# libdew_core). Linking dew_core as well would put its exported C entry points in the binary TWICE --
# once from the spliced objects, once from the import library. ld tolerates that; MSVC does not, and
# says so as LNK2005 (dew_pump_create already defined in pump.cpp.obj).
target_link_libraries(dew_converter_objects PRIVATE fmt glm unordered_dense dew_render vio_objstore libzstd_static)
target_include_directories(dew_converter_objects PRIVATE $<TARGET_PROPERTY:dew_core,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_definitions(dew_converter_objects PRIVATE $<TARGET_PROPERTY:dew_core,INTERFACE_COMPILE_DEFINITIONS>)
target_link_libraries(dew_converter_objects PRIVATE laszip)
target_include_directories(dew_converter_objects
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(dew_converter_objects
        PRIVATE
        ${libmorton_SOURCE_DIR}/include
        ${zstd_SOURCE_DIR}/lib
        ${zstd_SOURCE_DIR}/lib/common
)
setWarningflagsForTarget(dew_converter_objects)

if (MSVC)
    target_compile_options(dew_converter_objects PRIVATE /arch:AVX2)
else ()
    #target_compile_options(dew_converter_objects PRIVATE -mavx2)
endif ()

if (BUILD_SHARED_LIBS)
    target_compile_definitions(dew_converter_objects PRIVATE DEW_CONVERTER_EXPORTS)
    #    target_compile_options(dew_converter_objects PRIVATE -fsanitize=address)
    #    target_link_libraries(dew_converter_objects PRIVATE -fsanitize=address)
endif ()

# dew_core's objects are spliced in rather than linked as a library. Its cross-module API is
# internal C++ (decompress_any, the dataset format, the codecs) and the project builds with
# CMAKE_CXX_VISIBILITY_PRESET=hidden, so those symbols are not visible in libdew_core.dylib -- only
# the public dew_* C entry points are. Splicing objects is how tools/dew and tests already consume
# these modules; the shared libraries exist for the public C API.
add_library(dew_converter empty_dylib_function.cpp $<TARGET_OBJECTS:dew_converter_objects> $<TARGET_OBJECTS:dew_core_objects>)
#target_compile_options(dew_converter PRIVATE -fsanitize=address)
#target_link_options(dew_converter PRIVATE -fsanitize=address)
target_link_libraries(dew_converter PUBLIC dew_converter_objects)
target_link_libraries(dew_converter PRIVATE laszip)
