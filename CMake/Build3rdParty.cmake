macro(Build3rdParty)
    # Save and strip -fno-exceptions/-fno-rtti for third-party subdirectories
    # that need exceptions (fmt, laszip, etc). Restore after.
    set(_SAVED_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
    string(REPLACE "-fno-exceptions" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-fno-rtti" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    # MSVC equivalents: restore /EHsc (exceptions) and /GR (RTTI), remove _HAS_EXCEPTIONS=0
    string(REPLACE "/EHs-c-" "/EHsc" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "/GR-" "/GR" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REPLACE "-D_HAS_EXCEPTIONS=0" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

    if (DEW_BUILD_EXAMPLES)
        CmDepAddPackage(sdl OPTIONS SDL_TEST_LIBRARY=OFF)
    endif ()

    # Everything below is linked statically into the dew libraries. BUILD_SHARED_LIBS is shadowed with
    # a normal variable rather than forced through OPTIONS, which would unset the user's cache entry.
    # vio has its own VIO_BUILD_SHARED, but BoringSSL (vio's dependency) follows BUILD_SHARED_LIBS.
    set(_SAVED_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    # vio keeps its include directory PRIVATE; PUBLIC_INCLUDE re-exposes it.
    CmDepAddPackage(vio CONFIG PUBLIC_INCLUDE src
        OPTIONS VIO_BUILD_TESTS=OFF VIO_BUILD_EXAMPLES=OFF VIO_BUILD_GUI=OFF VIO_BUILD_SHARED=OFF VIO_INSTALL=OFF)

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

    CmDepAddPackage(fmt)
    # Clang on MinGW doesn't properly auto-detect -fno-exceptions for fmt headers
    if (NOT MSVC)
        target_compile_definitions(fmt PUBLIC FMT_EXCEPTIONS=0)
    endif ()
    CmDepAddPackage(glm)
    CmDepAddPackage(doctest OPTIONS DOCTEST_WITH_TESTS=OFF)
    CmDepAddPackage(unordered_dense)
    # zstd's CMake project lives in build/cmake, not at the root, so it cannot go through
    # CmDepAddPackage.
    set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_STATIC ON CACHE BOOL "" FORCE)
    set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    if (EMSCRIPTEN)
        # No pthreads in the single-threaded wasm build.
        set(ZSTD_MULTITHREAD_SUPPORT OFF CACHE BOOL "" FORCE)
    endif ()
    add_subdirectory(${zstd_SOURCE_DIR}/build/cmake ${CMAKE_CURRENT_BINARY_DIR}/zstd_build SYSTEM EXCLUDE_FROM_ALL)
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
        CmDepAddPackage(zlib_ng OPTIONS
            ZLIB_COMPAT=OFF ZLIB_ALIASES=OFF
            WITH_GZFILEOP=OFF   # we inflate memory buffers; no gzFile needed
            BUILD_TESTING=OFF ZLIB_ENABLE_TESTS=OFF WITH_GTEST=OFF
            WITH_BENCHMARKS=OFF WITH_BENCHMARK_APPS=OFF INSTALL_UTILS=OFF)
    endif ()

    set(BUILD_SHARED_LIBS ${_SAVED_BUILD_SHARED_LIBS})
    # CMakeRC is a CMake module, not a project: include() it rather than add it.
    include(${cmakerc_SOURCE_DIR}/CMakeRC.cmake)

    # Restore original flags for points' own code
    set(CMAKE_CXX_FLAGS ${_SAVED_CMAKE_CXX_FLAGS})
endmacro()
