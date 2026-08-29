macro(Build3rdParty)
    #list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake/FindPackage/SDL2)
    #GetPackageInstallDir(SDL2_INSTALL_DIR sdl_build ${sdl_VERSION})
    #Find_Package(SDL2 REQUIRED)
    #BuildExternalCMake(sdl_build ${sdl_VERSION} ${sdl_SOURCE_DIR} "" "SDL2::SDL2;SDL2::SDL2main")
    # Save and strip -fno-exceptions/-fno-rtti for third-party subdirectories
    # that need exceptions (vio tests, fmt, laszip, etc). Restore after.
    set(_SAVED_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
    string(REPLACE "-fno-exceptions" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-fno-rtti" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    # MSVC equivalents: restore /EHsc (exceptions) and /GR (RTTI), remove _HAS_EXCEPTIONS=0
    string(REPLACE "/EHs-c-" "/EHsc" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "/GR-" "/GR" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-D_HAS_EXCEPTIONS=0" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

    if (DEW_BUILD_EXAMPLES)
        set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
        add_subdirectory(${sdl_SOURCE_DIR} SYSTEM)
        unset(SDL_TEST_LIBRARY CACHE)
    endif ()

    set(VIO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(VIO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    # vio builds libuv as static (LIBUV_BUILD_SHARED OFF) but libuv's alias
    # target selects uv vs uv_a based on BUILD_SHARED_LIBS. Override it so
    # the alias points to the static target that actually gets built.
    set(_SAVED_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    add_subdirectory(${vio_SOURCE_DIR} SYSTEM)
    set(BUILD_SHARED_LIBS ${_SAVED_BUILD_SHARED_LIBS})
    unset(VIO_BUILD_TESTS CACHE)
    unset(VIO_BUILD_EXAMPLES CACHE)
    # vio changed its include directory from PUBLIC to PRIVATE; re-expose it
    target_include_directories(vio PUBLIC ${vio_SOURCE_DIR}/src)

    # laszip is only needed for LAS/LAZ ingest (the write/convert path), which the WebAssembly read-only
    # data library does not build. Skip it under Emscripten.
    if (NOT EMSCRIPTEN)
    # Build laszip as an OBJECT library directly from sources
    set(LASZIP_API_VERSION_MAJOR 3)
    set(LASZIP_API_VERSION_MINOR 5)
    set(LASZIP_API_VERSION_PATCH 1)
    set(HAVE_UNORDERED_MAP 1)
    set(_laszip_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/laszip_generated")
    configure_file(
        "${laszip_SOURCE_DIR}/laszip_api_version.h.in"
        "${_laszip_generated_dir}/laszip_api_version.h"
    )
    add_library(laszip OBJECT
        ${laszip_SOURCE_DIR}/src/mydefs.cpp
        ${laszip_SOURCE_DIR}/src/arithmeticdecoder.cpp
        ${laszip_SOURCE_DIR}/src/arithmeticencoder.cpp
        ${laszip_SOURCE_DIR}/src/arithmeticmodel.cpp
        ${laszip_SOURCE_DIR}/src/integercompressor.cpp
        ${laszip_SOURCE_DIR}/src/lasindex.cpp
        ${laszip_SOURCE_DIR}/src/lasinterval.cpp
        ${laszip_SOURCE_DIR}/src/lasmessage.cpp
        ${laszip_SOURCE_DIR}/src/lasquadtree.cpp
        ${laszip_SOURCE_DIR}/src/lasreadpoint.cpp
        ${laszip_SOURCE_DIR}/src/lasreaditemcompressed_v1.cpp
        ${laszip_SOURCE_DIR}/src/lasreaditemcompressed_v2.cpp
        ${laszip_SOURCE_DIR}/src/lasreaditemcompressed_v3.cpp
        ${laszip_SOURCE_DIR}/src/lasreaditemcompressed_v4.cpp
        ${laszip_SOURCE_DIR}/src/laswritepoint.cpp
        ${laszip_SOURCE_DIR}/src/laswriteitemcompressed_v1.cpp
        ${laszip_SOURCE_DIR}/src/laswriteitemcompressed_v2.cpp
        ${laszip_SOURCE_DIR}/src/laswriteitemcompressed_v3.cpp
        ${laszip_SOURCE_DIR}/src/laswriteitemcompressed_v4.cpp
        ${laszip_SOURCE_DIR}/src/laszip.cpp
        ${laszip_SOURCE_DIR}/src/laszip_dll.cpp
    )
    target_include_directories(laszip SYSTEM PUBLIC
        ${laszip_SOURCE_DIR}/dll
        ${laszip_SOURCE_DIR}/include/laszip
        ${_laszip_generated_dir}
    )
    target_include_directories(laszip PRIVATE
        ${laszip_SOURCE_DIR}/src
    )
    target_compile_definitions(laszip PRIVATE UNORDERED HAVE_UNORDERED_MAP=1 LASZIPDLL_EXPORTS)
    if (WIN32)
        target_compile_definitions(laszip PRIVATE NOMINMAX)
    endif ()
    if (MSVC)
        target_compile_options(laszip PRIVATE /w /EHsc)
    else ()
        target_compile_options(laszip PRIVATE -w -fexceptions)
    endif ()
    endif () # NOT EMSCRIPTEN (laszip)

    set(OLD_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    add_subdirectory(${fmt_SOURCE_DIR} SYSTEM)
    # Clang on MinGW doesn't properly auto-detect -fno-exceptions for fmt headers
    if (NOT MSVC)
        target_compile_definitions(fmt PUBLIC FMT_EXCEPTIONS=0)
    endif ()
    add_subdirectory(${glm_SOURCE_DIR} SYSTEM)
    set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${doctest_SOURCE_DIR} SYSTEM)
    unset(DOCTEST_WITH_TESTS CACHE)
    add_subdirectory(${unordered_dense_SOURCE_DIR} SYSTEM)
    set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_STATIC ON CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    if (EMSCRIPTEN)
        # No pthreads in the single-threaded wasm build.
        set(ZSTD_MULTITHREAD_SUPPORT OFF CACHE BOOL "" FORCE)
    endif ()
    add_subdirectory(${zstd_SOURCE_DIR}/build/cmake ${CMAKE_CURRENT_BINARY_DIR}/zstd_build SYSTEM)
    unset(ZSTD_BUILD_PROGRAMS CACHE)
    unset(ZSTD_BUILD_SHARED CACHE)
    unset(ZSTD_BUILD_STATIC CACHE)
    unset(ZSTD_BUILD_TESTS CACHE)
    # zlib-ng, for examples/ford. Built in its NATIVE mode (ZLIB_COMPAT off): the header is
    # <zlib-ng.h> and the symbols are zng_*, so it cannot collide with a system zlib that something
    # else in the link drags in. ZLIB_ALIASES would add plain `zlib`/`zlibstatic` target names for
    # drop-in consumers; off, because a global target called `zlib` is exactly the kind of name that
    # collides later. BUILD_TESTING defaults ON here and pulls gtest, so it must be forced off.
    if (DEW_BUILD_EXAMPLES)
        set(ZLIB_COMPAT OFF CACHE BOOL "" FORCE)
        set(ZLIB_ALIASES OFF CACHE BOOL "" FORCE)
        set(WITH_GZFILEOP OFF CACHE BOOL "" FORCE)   # we inflate memory buffers; no gzFile needed
        set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
        set(ZLIB_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
        set(WITH_GTEST OFF CACHE BOOL "" FORCE)
        set(WITH_BENCHMARKS OFF CACHE BOOL "" FORCE)
        set(WITH_BENCHMARK_APPS OFF CACHE BOOL "" FORCE)
        set(INSTALL_UTILS OFF CACHE BOOL "" FORCE)
        add_subdirectory(${zlib_ng_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR}/zlib_ng_build SYSTEM)
        unset(ZLIB_COMPAT CACHE)
        unset(ZLIB_ALIASES CACHE)
        unset(WITH_GZFILEOP CACHE)
        unset(BUILD_TESTING CACHE)
        unset(ZLIB_ENABLE_TESTS CACHE)
        unset(WITH_GTEST CACHE)
        unset(WITH_BENCHMARKS CACHE)
        unset(WITH_BENCHMARK_APPS CACHE)
        unset(INSTALL_UTILS CACHE)
    endif ()

    set(BUILD_SHARED_LIBS ${OLD_BUILD_SHARED_LIBS})
    include(${cmakerc_SOURCE_DIR}/CMakeRC.cmake)

    # Restore original flags for points' own code
    set(CMAKE_CXX_FLAGS ${_SAVED_CMAKE_CXX_FLAGS})
endmacro()
