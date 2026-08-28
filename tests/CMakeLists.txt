add_library(doctest_main STATIC doctest-main.cpp)
target_link_libraries(doctest_main PUBLIC doctest)
target_compile_definitions(doctest_main PUBLIC DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS)

add_executable(public_interface_unit_tests "public/renderer_base.cpp")
target_link_libraries(public_interface_unit_tests PRIVATE doctest_main doctest fmt dew_render)

add_executable(private_interface_unit_tests
        private/basic_camera.cpp
        private/morton_tests.cpp
        private/tree_tests.cpp
        private/vector_updater_tests.cpp
        private/fixed_size_vector_tests.cpp
        private/converter_tests.cpp
        private/deque_map_test.cpp
        private/blob_manager_test.cpp
        private/blob_residency_test.cpp
        private/cached_upload_tests.cpp
        private/storage_backend_test.cpp
        private/cloud_io_live_test.cpp
        $<TARGET_OBJECTS:dew_core_objects>
        $<TARGET_OBJECTS:dew_converter_objects>
        $<TARGET_OBJECTS:dew_render_objects>
        private/frustum.cpp
        private/byte_shuffle_tests.cpp
        private/compression_preprocess_tests.cpp
        private/compressor_tests.cpp
        private/camera_arcball_tests.cpp
        private/memory_writer_tests.cpp
        private/memory_budget_tests.cpp
        private/access_snapshot_tests.cpp
        private/blob_reader_tests.cpp
        private/tree_set_tests.cpp
        private/render_callback_tests.cpp
        private/cpp_await_tests.cpp
        private/dewpp_tests.cpp
        private/pump_tests.cpp
        private/access_query_tests.cpp
        private/converter_teardown_tests.cpp
        private/attribute_amend_tests.cpp
        $<TARGET_OBJECTS:dew_access_objects>
)
target_link_libraries(private_interface_unit_tests PRIVATE dew::await vio_objstore libzstd_static)
target_link_libraries(private_interface_unit_tests PRIVATE doctest_main doctest fmt glm unordered_dense laszip)
target_include_directories(private_interface_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src/render ${libmorton_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/core ${PROJECT_SOURCE_DIR}/src/render ${PROJECT_SOURCE_DIR}/src/converter ${PROJECT_SOURCE_DIR}/src/access ${libmorton_SOURCE_DIR}/include)
copy_dll_for_target(public_interface_unit_tests dew_render dew_converter)
copy_dll_for_target(private_interface_unit_tests dew_render dew_converter)
