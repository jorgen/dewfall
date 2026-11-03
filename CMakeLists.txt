cmake_minimum_required(VERSION 3.30)
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

project(dewfall VERSION 0.1.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

include(FetchContent)
FetchContent_Declare(cmake-dep
    GIT_REPOSITORY https://github.com/jorgen/cmake-dep.git
    GIT_TAG 756bee3072d1bf67e179b62a9635cdcdef5931e7
)
FetchContent_MakeAvailable(cmake-dep)
list(APPEND CMAKE_MODULE_PATH "${cmake-dep_SOURCE_DIR}/cmake")
include(CmDepMain)

include(GNUInstallDirs)
include(GenerateExportHeader)

get_property(LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)
if ("${LIB64}" STREQUAL "TRUE" AND NOT FORCE_NO_LIBDIR_SUFFIX)
    set(LIBSUFFIX 64)
else ()
    set(LIBSUFFIX "")
endif ()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (WIN32)
    # _CRT_SECURE_NO_WARNINGS: the CRT deprecates std::getenv in favour of the
    # non-portable _dupenv_s, and the 17 getenv call sites here are deliberate
    # (debug-hook and cache-toggle environment lookups). With /W4 /WX that
    # deprecation is a hard build failure on MSVC.
    add_compile_definitions(NOMINMAX=1 WIN32_LEAN_AND_MEAN=1 VC_EXTRALEAN=1 _CRT_SECURE_NO_WARNINGS=1)
endif ()
if (MSVC)
    string(REGEX REPLACE "/GR" "/GR-" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REGEX REPLACE "/EHsc" "/EHs-c-" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_HAS_EXCEPTIONS=0")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")
endif ()

if (WIN32)
    if (CMAKE_VS_PLATFORM_TOOLSET)
        string(REGEX REPLACE "^v" "" MSVC_TOOLSET_VERSION_LOCAL "${CMAKE_VS_PLATFORM_TOOLSET}")
    else ()
        set(MSVC_TOOLSET_VERSION_LOCAL ${MSVC_TOOLSET_VERSION})
    endif ()
endif ()

option(DEW_BUILD_TESTS "Build tests" ON)
option(DEW_BUILD_EXAMPLES "Build examples" ON)
option(DEW_BUILD_PYTHON "Build the dew Python bindings (needs python3 + pip libclang)" OFF)
option(BUILD_SHARED_LIBS "Build shared libs" ON)

if(EMSCRIPTEN)
    # The WebAssembly build produces a single read-only data library (src/wasm); no shared libraries,
    # tests, or examples, and no laszip/SDL. Everything below is guarded by EMSCRIPTEN so the native
    # build is unaffected.
    set(DEW_BUILD_TESTS OFF)
    set(DEW_BUILD_EXAMPLES OFF)
    set(BUILD_SHARED_LIBS OFF)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
find_package(Threads)

include(CMake/SetWarnings.cmake)

set(CMDEP_PACKAGES_FILE "${CMAKE_CURRENT_SOURCE_DIR}/CMake/3rdPartyPackages.cmake")
CmDepFetch()

include(CMake/Build3rdParty.cmake)
Build3rdParty()
include(CMake/AddSystemHeadersToTarget.cmake)
include(CMake/CopyDllForTarget.cmake)

add_subdirectory(src)

if (DEW_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()

if (DEW_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

# The dew CLI depends on the native converter/render libraries.
if (NOT EMSCRIPTEN)
    add_subdirectory(tools)
endif ()

# Header-only and checked in, so it needs no generator at build time and is always available.
add_subdirectory(bindings/cpp)

if (DEW_BUILD_PYTHON AND NOT EMSCRIPTEN)
    add_subdirectory(bindings/python)
endif ()

if (NOT DEFINED CPACK_GENERATOR)
    if (UNIX)
        set(CPACK_GENERATOR "TGZ")
    elseif (WIN32)
        set(CPACK_GENERATOR "ZIP")
    endif ()
endif ()

if (UNIX)
    find_program(LSB_RELEASE_EXEC lsb_release)
    if (LSB_RELEASE_EXEC)
        execute_process(COMMAND ${LSB_RELEASE_EXEC} -irs
                OUTPUT_VARIABLE OS_NAME
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    else ()
        set(OS_NAME "linux")
    endif ()
else ()
    # Always set OS_NAME to "win" for windows
    set(OS_NAME "win-${MSVC_TOOLSET_VERSION_LOCAL}")
endif ()

if (NOT DEFINED PACKAGE_RELEASE)
    set(PACKAGE_RELEASE "1")
endif ()

set(CPACK_PACKAGE_VERSION "${CMAKE_PROJECT_VERSION}")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}/package-install")
set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${CMAKE_PROJECT_VERSION}-${PACKAGE_RELEASE}-${OS_NAME}")

include(CPack)

