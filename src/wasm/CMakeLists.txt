# Read-only point-cloud data library for WebAssembly. Exposes (via embind) an API to open an S3 dataset
# with caller-supplied credentials, load blobs over emscripten_fetch, and return DECOMPRESSED buffers.
# Emits dew_data.mjs + dew_data.wasm (an ES6 module). Only built under Emscripten.

set(_conv ${CMAKE_CURRENT_SOURCE_DIR}/../converter)
set(_core ${CMAKE_CURRENT_SOURCE_DIR}/../core)
set(_render ${CMAKE_CURRENT_SOURCE_DIR}/../render)
set(_access ${CMAKE_CURRENT_SOURCE_DIR}/../access)

add_executable(dew_data_wasm
    dew_wasm_api.cpp
    ${_core}/error.cpp
    ${_core}/attributes_api.cpp
    ${_core}/pump.cpp
    # The portable read + decompress subset (no libuv / laszip):
    ${_core}/object_backend.cpp
    ${_core}/bucket_format.cpp
    ${_core}/compressor.cpp
    ${_core}/compressor_zstd.cpp
    ${_core}/compressor_fse.cpp
    ${_core}/compressor_ans.cpp
    ${_core}/byte_shuffle.cpp
    ${_core}/compression_preprocess.cpp
    # Tree (de)serialize + node storage map + attribute configs, for the frustum-walk / readNode path:
    ${_core}/tree.cpp
    ${_core}/tree_set.cpp
    ${_core}/input_storage_map.cpp
    ${_core}/attributes_configs.cpp
)

target_include_directories(dew_data_wasm PRIVATE
    ${_conv}
    ${_core}
    ${_render}
    ${fmt_SOURCE_DIR}/include
    ${glm_SOURCE_DIR}
    ${libmorton_SOURCE_DIR}/include
    ${unordered_dense_SOURCE_DIR}/include
    ${zstd_SOURCE_DIR}/lib
)

# FMT_HEADER_ONLY avoids building the fmt library under emcc; ZSTD_DISABLE_ASM skips zstd's .S paths.
target_compile_definitions(dew_data_wasm PRIVATE FMT_HEADER_ONLY=1 ZSTD_DISABLE_ASM=1)

# embind requires RTTI (the top-level build sets -fno-rtti globally); fmt 11 references malloc/free
# without pulling <cstdlib> under emscripten's clang, so force-include it. Our own code stays
# -fno-exceptions (which is what embind's type registration expects here), but vio is compiled with
# exceptions and references __cxa_throw, so the throwing runtime must still be linked in
# (-sDISABLE_EXCEPTION_THROWING=0) even though we don't throw ourselves.
# --bind is needed at COMPILE time too (not only link) for embind. -fvisibility=default overrides the
# project-wide CXX_VISIBILITY_PRESET=hidden: hidden visibility keeps the per-TU RTTI type_info symbols
# from merging with embind's runtime, which makes every bound function report "unbound types" at call time.
target_compile_options(dew_data_wasm PRIVATE -frtti -include cstdlib --bind -fvisibility=default)

target_link_libraries(dew_data_wasm PRIVATE vio libzstd_static)

target_link_options(dew_data_wasm PRIVATE
    # RTTI must also be on at LINK: emscripten (re)builds its system libs -- including libembind -- with
    # the link-command flags, and the global -fno-rtti would build libembind without RTTI, so its
    # type_info for int/val/etc. would not match our -frtti TUs -> "unbound types" for every function.
    -frtti
    "-sDISABLE_EXCEPTION_THROWING=0"
    --bind
    "-sFETCH=1"
    "-sASYNCIFY"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sMAXIMUM_MEMORY=1GB"
    "-sEXIT_RUNTIME=0"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sEXPORT_NAME=DewData"
    "-sENVIRONMENT=web,worker,node"
    "-sSTACK_SIZE=5MB"
)

set_target_properties(dew_data_wasm PROPERTIES OUTPUT_NAME "dew_data" SUFFIX ".mjs")

# ---------------------------------------------------------------------------------------------------
# WebGL2 renderer library. Streams + decodes the octree and draws it with the desktop gl_renderer
# (ported to GLES3), driven by a React canvas + a semantic arcball camera API. Emits
# dew_render.mjs + .wasm (an ES6 module). Rendering is dirty-driven: WASM calls a JS requestUpdate
# callback (input / async-IO completion) instead of a continuous main loop.
set(_examples ${CMAKE_SOURCE_DIR}/examples/renderer)

# The GLSL sources, embedded via cmrc under the "shaders" namespace (matches gl_renderer's CMRC_DECLARE).
# WHENCE keeps the resource names as "shaders/xxx" even though the target lives in src/wasm.
cmrc_add_resource_library(dew_render_wasm_shaders ALIAS dew_render_wasm::shaders NAMESPACE shaders
    WHENCE ${_examples}
    ${_examples}/shaders/aabb.frag
    ${_examples}/shaders/aabb.vert
    ${_examples}/shaders/points.frag
    ${_examples}/shaders/points.vert
    ${_examples}/shaders/dynpoints.frag
    ${_examples}/shaders/dynpoints_1.vert
    ${_examples}/shaders/dynpoints_3.vert
    ${_examples}/shaders/dynpoints_crossfade.vert
    ${_examples}/shaders/node_bbox.vert
    ${_examples}/shaders/node_bbox.frag
    ${_examples}/shaders/axis_gizmo.vert
    ${_examples}/shaders/axis_gizmo.frag
    ${_examples}/shaders/origin_anchor.vert
    ${_examples}/shaders/origin_anchor.frag
    ${_examples}/shaders/environment.vert
    ${_examples}/shaders/environment.frag
    ${_examples}/shaders/skybox.vert
    ${_examples}/shaders/skybox.frag
)
# cmrc.hpp calls abort() without <cstdlib>; emscripten's strict headers need it force-included (the
# generated cmrc TU does not inherit the main target's -include cstdlib).
target_compile_options(dew_render_wasm_shaders PRIVATE -include cstdlib)

add_executable(dew_render_wasm
    ${_examples}/renderer_wasm.cpp
    ${_examples}/gl_renderer.cpp
    # dew_render (pure C++/glm; no GL, no uv)
    ${_render}/renderer.cpp
    ${_render}/camera.cpp
    ${_render}/data_source.cpp
    ${_render}/data_source_aabb.cpp
    ${_render}/data_source_skybox.cpp
    ${_render}/data_source_axis_gizmo.cpp
    ${_render}/data_source_origin_anchor.cpp
    ${_render}/data_source_environment.cpp
    ${_render}/buffer.cpp
    ${_render}/image_decoder.cpp
    # converter read/stream path (the write/laszip/packed-file TUs are excluded)
    ${_conv}/processor.cpp
    ${_conv}/attribute_amend.cpp # processor_t owns an attribute_amend_t
    ${_conv}/storage_handler.cpp
    ${_core}/blob_reader.cpp
    ${_conv}/tree_handler.cpp
    ${_conv}/tree_collapse.cpp
    ${_conv}/tree_lod_generator.cpp
    ${_core}/tree.cpp
    ${_core}/tree_set.cpp
    # Tree CONSTRUCTION, needed only because this target compiles tree_handler.cpp (the write
    # pipeline) to reach read access through processor_t. The data + decode-worker modules link
    # tree.cpp alone. Drops out once the streaming data source stops embedding a processor_t.
    ${_conv}/tree_build.cpp
    ${_conv}/sorter.cpp
    ${_conv}/reader.cpp
    ${_conv}/input_header.cpp
    ${_core}/attributes_configs.cpp
    ${_conv}/data_source_converter.cpp
    ${_conv}/data_source_node_bbox.cpp
    ${_conv}/render_pipeline.cpp
    ${_conv}/virtual_tree.cpp
    ${_conv}/frustum_tree_walker.cpp
    ${_conv}/native_node_data_loader.cpp
    ${_conv}/worker_node_data_loader.cpp
    ${_conv}/node_decode.cpp
    ${_core}/blob_manager.cpp
    ${_conv}/input_data_source_registry.cpp
    ${_core}/input_storage_map.cpp
    ${_core}/storage_backend.cpp
    ${_core}/object_backend.cpp
    ${_core}/bucket_format.cpp
    ${_core}/compressor.cpp
    ${_core}/compressor_zstd.cpp
    ${_core}/compressor_fse.cpp
    ${_core}/compressor_ans.cpp
    ${_core}/byte_shuffle.cpp
    ${_core}/compression_preprocess.cpp
    ${_core}/error.cpp
    ${_core}/attributes_api.cpp
    ${_core}/pump.cpp
)

target_include_directories(dew_render_wasm PRIVATE
    ${_examples}
    ${_conv}
    ${_core}
    ${_render}
    ${fmt_SOURCE_DIR}/include
    ${glm_SOURCE_DIR}
    ${libmorton_SOURCE_DIR}/include
    ${unordered_dense_SOURCE_DIR}/include
    ${zstd_SOURCE_DIR}/lib
    ${stbimage_SOURCE_DIR}
)

target_compile_definitions(dew_render_wasm PRIVATE FMT_HEADER_ONLY=1 ZSTD_DISABLE_ASM=1)
target_compile_options(dew_render_wasm PRIVATE -frtti -include cstdlib --bind -fvisibility=default)

target_link_libraries(dew_render_wasm PRIVATE vio libzstd_static dew_render_wasm::shaders)

target_link_options(dew_render_wasm PRIVATE
    -frtti
    "-sDISABLE_EXCEPTION_THROWING=0"
    --bind
    "-sFETCH=1"
    "-sASYNCIFY"
    "-sALLOW_MEMORY_GROWTH=1"
    # Hard heap ceiling (default would be 2GB -- past what mobile browsers survive). ABORTING_MALLOC
    # defaults on, so growth past the ceiling is a clean wasm trap, never corrupted state (we build
    # -fno-exceptions; bad_alloc recovery is impossible anyway). The data source's heap-pressure brake
    # (memory_budget.hpp) tightens IO/cache caps at 80%/90% of this ceiling so the trap is unreachable in
    # practice; INITIAL_MEMORY skips the first ~dozen growth events on the common path.
    "-sMAXIMUM_MEMORY=1GB"
    "-sINITIAL_MEMORY=128MB"
    "-sEXIT_RUNTIME=0"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sEXPORT_NAME=DewRender"
    "-sENVIRONMENT=web,worker,node"
    "-sSTACK_SIZE=8MB"
    # WebGL2 / GLES3 (FULL_ES3 also enables glMapBufferRange + client-array emulation).
    "-sMAX_WEBGL_VERSION=2"
    "-sMIN_WEBGL_VERSION=2"
    "-sFULL_ES3=1"
)

set_target_properties(dew_render_wasm PROPERTIES OUTPUT_NAME "dew_render" SUFFIX ".mjs")

# ---------------------------------------------------------------------------------------------------------
# Pure-CPU decode Web Worker module (decompress -> deserialize -> decode_node). No WebGL, no FETCH, no
# ASYNCIFY -- it only crunches bytes handed to it over postMessage (see examples/renderer/web/src/decodeWorker
# .ts + decodeWorkerPool.ts, and the as-built notes in decode_worker.cpp). Built + node-smoke-tested under
# EMSDK; the worker_node_data_loader (src/converter) drives it.
add_executable(dew_decode_worker
    decode_worker.cpp
    ${_core}/error.cpp
    ${_core}/attributes_api.cpp
    ${_core}/pump.cpp
    ${_conv}/node_decode.cpp            # the extracted decode seam
    ${_core}/compressor.cpp
    ${_core}/compressor_zstd.cpp
    ${_core}/compressor_fse.cpp
    ${_core}/compressor_ans.cpp
    ${_core}/byte_shuffle.cpp
    ${_core}/compression_preprocess.cpp
    ${_core}/tree.cpp                   # tree/registry (de)serialize -- format only, no storage deps
    ${_core}/input_storage_map.cpp
    ${_core}/attributes_configs.cpp
)
target_include_directories(dew_decode_worker PRIVATE
    ${_conv} ${_core} ${_render}
    ${fmt_SOURCE_DIR}/include ${glm_SOURCE_DIR} ${libmorton_SOURCE_DIR}/include
    ${unordered_dense_SOURCE_DIR}/include ${zstd_SOURCE_DIR}/lib
)
target_compile_definitions(dew_decode_worker PRIVATE FMT_HEADER_ONLY=1 ZSTD_DISABLE_ASM=1)
target_compile_options(dew_decode_worker PRIVATE -frtti -include cstdlib --bind -fvisibility=default)
target_link_libraries(dew_decode_worker PRIVATE vio libzstd_static)
target_link_options(dew_decode_worker PRIVATE
    -frtti
    "-sDISABLE_EXCEPTION_THROWING=0"
    --bind
    "-sALLOW_MEMORY_GROWTH=1"          # note: NO -sFETCH / NO -sASYNCIFY -- pure CPU
    # Each worker owns an independent heap; per-job peak is bounded by the largest node (compressed +
    # decompressed + decode outputs + salvage clones), well under 256MB. See the render module's ceiling
    # comment for the ABORTING_MALLOC semantics.
    "-sMAXIMUM_MEMORY=256MB"
    "-sINITIAL_MEMORY=64MB"
    "-sEXIT_RUNTIME=0"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sEXPORT_NAME=DewDecodeWorker"
    "-sENVIRONMENT=web,worker,node"
    "-sSTACK_SIZE=5MB"
)
set_target_properties(dew_decode_worker PROPERTIES OUTPUT_NAME "dew_decode_worker" SUFFIX ".mjs")

# ---------------------------------------------------------------------------------------------------------
# Proof that the query engine never blocks: dew_access built and RUN with no ASYNCIFY.
#
# dew_access is meant to be embeddable in a module that has none -- a decode worker, a service worker,
# anything driven from inside a JS callback that cannot tolerate a stack unwind. ASYNCIFY is a
# whole-program transform (~40-50% code size, viral, un-re-enterable from a callback), so "does not
# require it" is worth holding rather than assuming.
#
# This is NOT a link test, though it started as one. "A blocking read calls emscripten_sleep, which
# without -sASYNCIFY is a link error" is false: emscripten resolves it to an aborting stub. And the
# module imports emscripten_sleep regardless, because storage_backend_t's synchronous exists() /
# read_index() are virtual and object_backend_t's overrides route through run_on_loop_blocking -- a
# retained vtable keeps them reachable even though access only calls the _async forms.
#
# So the gate is the node smoke test: it opens a dataset and drives it to a terminal state through an
# ordinary poll loop, against a stubbed XHR that fails on a later turn. The aborting stub makes any
# blocking call fail loudly at run time. See access_noasyncify_probe.cpp for the full reasoning.
add_executable(dew_access_noasyncify
    access_noasyncify_probe.cpp
    ${_core}/error.cpp
    ${_core}/attributes_api.cpp
    ${_core}/pump.cpp
    ${_core}/blob_reader.cpp
    ${_core}/storage_backend.cpp
    ${_core}/object_backend.cpp
    ${_core}/bucket_format.cpp
    ${_core}/blob_manager.cpp
    ${_core}/compressor.cpp
    ${_core}/compressor_zstd.cpp
    ${_core}/compressor_fse.cpp
    ${_core}/compressor_ans.cpp
    ${_core}/byte_shuffle.cpp
    ${_core}/compression_preprocess.cpp
    ${_core}/tree.cpp
    ${_core}/tree_set.cpp
    ${_core}/input_storage_map.cpp
    ${_core}/attributes_configs.cpp
    ${_access}/region_walk.cpp
    ${_access}/decode.cpp
    ${_access}/dataset.cpp
    ${_access}/request.cpp
    ${_access}/query_api.cpp
)
target_include_directories(dew_access_noasyncify PRIVATE
    ${_access} ${_core} ${_render}
    ${fmt_SOURCE_DIR}/include ${glm_SOURCE_DIR} ${libmorton_SOURCE_DIR}/include
    ${unordered_dense_SOURCE_DIR}/include ${zstd_SOURCE_DIR}/lib
)
target_compile_definitions(dew_access_noasyncify PRIVATE FMT_HEADER_ONLY=1 ZSTD_DISABLE_ASM=1)
target_compile_options(dew_access_noasyncify PRIVATE -include cstdlib)
# The probe's own EMSCRIPTEN_KEEPALIVE entry points retain everything they CALL; this list retains the
# rest of the public surface so the build covers the request path too, not only open. Without some
# form of forced retention the linker strips the lot -- an earlier version referenced the functions
# through a volatile address array, which the compiler folded away, leaving a 60-byte module that
# "passed" while containing no engine at all.
set(_access_probe_exports
    _dew_dataset_create,_dew_dataset_close,_dew_dataset_state,_dew_dataset_wait_ready
    ,_dew_dataset_poll,_dew_dataset_pending_count,_dew_dataset_get_info
    ,_dew_dataset_attribute_count,_dew_dataset_get_attribute_name
    ,_dew_dataset_request_region,_dew_request_status,_dew_request_wait,_dew_request_cancel
    ,_dew_request_get_result,_dew_request_attribute_size,_dew_request_copy_attribute
    ,_dew_request_release,_dew_pump_create,_dew_pump_destroy,_dew_pump_poll
    ,_dew_pump_set_wake_callback,_main)
string(REPLACE ";" "" _access_probe_exports "${_access_probe_exports}")
target_link_libraries(dew_access_noasyncify PRIVATE vio libzstd_static)
target_link_options(dew_access_noasyncify PRIVATE
    "-sDISABLE_EXCEPTION_THROWING=0"
    # FETCH yes, ASYNCIFY deliberately NOT. They are different things and only the second is the
    # problem: -sFETCH is an ordinary library (the object backend's HTTP transport, used purely in its
    # callback-driven async mode), while ASYNCIFY rewrites the whole program. Reaching an object store
    # over https is the point of this module; unwinding the stack to wait for it is what is banned.
    "-sFETCH=1"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sEXIT_RUNTIME=0"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sEXPORT_NAME=DewAccessProbe"
    "-sENVIRONMENT=web,worker,node"
    "-sEXPORTED_FUNCTIONS=${_access_probe_exports}"
    "-sEXPORTED_RUNTIME_METHODS=UTF8ToString"   # the smoke harness reads the dataset's error text
)
set_target_properties(dew_access_noasyncify PROPERTIES OUTPUT_NAME "dew_access_probe" SUFFIX ".mjs")
