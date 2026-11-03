
set(public_headers
        dew/core/error.h
        dew/core/export.h
        dew/core/format.h
        dew/core/types.h
        dew/core/pump.h
        dew/core/default_attribute_names.h
)
set(private_headers
        format_util.hpp
        pump.hpp
        dataset_types.hpp
        morton.hpp
        morton_tree_coordinate_transform.hpp
        lru_cache.hpp
        memory_writer.hpp
        perf_stats.hpp
        url.hpp
        index_format.hpp
        compressor.hpp
        compressor_zstd.hpp
        compressor_fse.hpp
        compressor_ans.hpp
        compression_preprocess.hpp
        byte_shuffle.hpp
        budget.hpp
        tree.hpp
        tree_set.hpp
        input_storage_map.hpp
        attributes_configs.hpp
        storage_backend.hpp
        object_backend.hpp
        packed_file_backend.hpp
        bucket_format.hpp
        blob_manager.hpp
        blob_reader.hpp
        blob_residency.hpp
        spill_store.hpp
        file_hole_punch.hpp
        error.hpp
        fixed_size_vector.hpp
)
set(sources
        error.cpp
        attributes_api.cpp
        pump.cpp
        compressor.cpp
        compressor_zstd.cpp
        compressor_fse.cpp
        compressor_ans.cpp
        compression_preprocess.cpp
        byte_shuffle.cpp
        tree.cpp
        tree_set.cpp
        input_storage_map.cpp
        attributes_configs.cpp
        storage_backend.cpp
        object_backend.cpp
        packed_file_backend.cpp
        bucket_format.cpp
        blob_manager.cpp
        blob_reader.cpp
        blob_residency.cpp
        spill_store.cpp
        file_hole_punch.cpp
)

add_library(dew_core_objects OBJECT ${public_headers} ${private_headers} ${sources})
# The core now carries the dataset format and the compression codecs, so it needs what they need:
# fmt (formatters on the dataset types), libmorton (the Z-order encode/decode kernels), zstd, and
# ankerl's dense map. vio supplies the event loop / task types the storage layer is built on.
target_link_libraries(dew_core_objects PRIVATE fmt glm unordered_dense vio_objstore libzstd_static)
target_include_directories(dew_core_objects
        PRIVATE
        ${libmorton_SOURCE_DIR}/include
        ${zstd_SOURCE_DIR}/lib
        ${zstd_SOURCE_DIR}/lib/common
)
add_library(dew_core empty_dylib_function.cpp $<TARGET_OBJECTS:dew_core_objects>)
target_link_libraries(dew_core PUBLIC dew_core_objects)
if (BUILD_SHARED_LIBS)
    target_compile_definitions(dew_core_objects PRIVATE DEW_CORE_EXPORTS)
endif ()
set_target_properties(dew_core_objects
        PROPERTIES
        PUBLIC_HEADER "${public_headers}"
)
target_include_directories(dew_core_objects
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
        $<INSTALL_INTERFACE:include>
)
