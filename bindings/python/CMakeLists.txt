# Python bindings: public headers -> JSON IR -> nanobind C++ -> `import dew`.
#
# The two generator stages (tools/bindgen/parse_headers.py + generate_nanobind.py)
# run at build time; nothing generated is committed. Requires a python3 with the
# `libclang` package (pip install libclang) -- point CMake at it with
# -DPython_EXECUTABLE=/path/to/python when it is not the default interpreter.

# nanobind needs RTTI and exceptions, which the project disables globally.
# Restore them at directory scope (same idiom as src/wasm and Build3rdParty);
# this also covers libnanobind and robin-map added below. Exceptions never
# cross into the dew libraries: wrappers throw only after the C call returns.
string(REPLACE "-fno-exceptions" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-fno-rtti" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "/EHs-c-" "/EHsc" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "/GR-" "/GR" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-D_HAS_EXCEPTIONS=0" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

# SKBUILD_SABI_COMPONENT is "Development.SABIModule" when scikit-build-core is
# targeting abi3, and empty otherwise -- so a plain cmake build is unaffected,
# while a wheel build gets the component nanobind needs. Without it nanobind
# silently declines the stable ABI while the wheel is still tagged abi3.
find_package(Python 3.10 REQUIRED
    COMPONENTS Interpreter Development.Module ${SKBUILD_SABI_COMPONENT})

# Does this build have to produce a genuine Py_LIMITED_API binary?
if (SKBUILD_SABI_VERSION AND NOT Python_SOABI MATCHES "[0-9]t")
    set(DEW_PYTHON_REQUIRE_STABLE_ABI ON)
else ()
    set(DEW_PYTHON_REQUIRE_STABLE_ABI OFF)
endif ()

execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import clang.cindex"
    RESULT_VARIABLE _dew_have_libclang
    OUTPUT_QUIET ERROR_QUIET)
if (NOT _dew_have_libclang EQUAL 0)
    message(FATAL_ERROR
        "DEW_BUILD_PYTHON: ${Python_EXECUTABLE} cannot import clang.cindex. "
        "Install it with: ${Python_EXECUTABLE} -m pip install libclang")
endif ()

# The fetched nanobind tarball lacks its ext/robin_map submodule; the separately
# fetched robin-map provides the tsl::robin_map target nanobind falls back to.
set(NB_USE_SUBMODULE_DEPS OFF CACHE BOOL "" FORCE)
add_subdirectory(${robin_map_SOURCE_DIR} robin_map SYSTEM)
add_subdirectory(${nanobind_SOURCE_DIR} nanobind SYSTEM)

set(_bindgen_dir ${CMAKE_SOURCE_DIR}/tools/bindgen)
set(_ir ${CMAKE_CURRENT_BINARY_DIR}/dew_api.json)
set(_generated ${CMAKE_CURRENT_BINARY_DIR}/dew_bindings_generated.cpp)

# Every public header in the manifest is a dependency of the IR. The DEPENDS
# list is fixed at configure time, so the manifest itself is a configure
# dependency: adding a header to it re-runs CMake and picks the new file up.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_bindgen_dir}/public_headers.txt)
file(STRINGS ${_bindgen_dir}/public_headers.txt _dew_manifest_lines)
set(_dew_public_headers "")
foreach (_line IN LISTS _dew_manifest_lines)
    string(REGEX REPLACE "#.*" "" _line "${_line}")
    string(STRIP "${_line}" _line)
    # '!' lines are acknowledged exclusions, not inputs -- but they still name
    # real headers (export.h, transitively included), so track them too.
    string(REGEX REPLACE "^!" "" _line "${_line}")
    if (_line)
        list(APPEND _dew_public_headers ${CMAKE_SOURCE_DIR}/src/${_line})
    endif ()
endforeach ()

add_custom_command(OUTPUT ${_ir}
    COMMAND ${Python_EXECUTABLE} ${_bindgen_dir}/parse_headers.py
            --source-root ${CMAKE_SOURCE_DIR}/src
            --manifest ${_bindgen_dir}/public_headers.txt
            --output ${_ir}
    DEPENDS ${_dew_public_headers}
            ${_bindgen_dir}/public_headers.txt
            ${_bindgen_dir}/parse_headers.py
            ${_bindgen_dir}/clang_util.py
            ${_bindgen_dir}/semantics.py
            ${_bindgen_dir}/ir.py
    VERBATIM
    COMMENT "bindgen: public headers -> dew_api.json")

add_custom_command(OUTPUT ${_generated}
    COMMAND ${Python_EXECUTABLE} ${_bindgen_dir}/generate_nanobind.py
            --ir ${_ir} --output ${_generated}
    DEPENDS ${_ir} ${_bindgen_dir}/generate_nanobind.py
    VERBATIM
    COMMENT "bindgen: dew_api.json -> nanobind C++")

if (DEW_PYTHON_REQUIRE_STABLE_ABI)
    # nanobind has four independent branches that silently set STABLE_ABI FALSE
    # (cmake/nanobind-config.cmake); scikit-build-core tags the wheel abi3
    # regardless, so a mislabelled version-locked binary is the default failure
    # mode. Turn each branch into a named error. (nanobind's own guard for this
    # is dead code: it tests SKBUILD_ABI_VERSION, missing the "SA".)
    if (Python_VERSION VERSION_LESS 3.12)
        message(FATAL_ERROR "abi3 wheel requested but Python is ${Python_VERSION}; nanobind's "
                            "limited-API support requires CPython >= 3.12")
    endif ()
    if (NOT Python_INTERPRETER_ID STREQUAL "Python")
        message(FATAL_ERROR "abi3 wheel requested but the interpreter is "
                            "${Python_INTERPRETER_ID}, not CPython")
    endif ()
    if (NOT TARGET Python::SABIModule)
        message(FATAL_ERROR "abi3 wheel requested but Python::SABIModule was not found "
                            "(SKBUILD_SABI_COMPONENT='${SKBUILD_SABI_COMPONENT}')")
    endif ()
    if (SKBUILD_SABI_VERSION VERSION_LESS 3.12)
        message(FATAL_ERROR "wheel.py-api must be cp312 or later for nanobind, got "
                            "${SKBUILD_SABI_VERSION}")
    endif ()
    nanobind_add_module(dew_python NB_STATIC STABLE_ABI ${_generated} dew_py_support.cpp
        dew_py_support.h custom/file_convert_callbacks.h custom/amend.h custom/query.h custom/query_async.h)
else ()
    nanobind_add_module(dew_python NB_STATIC ${_generated} dew_py_support.cpp
        dew_py_support.h custom/file_convert_callbacks.h custom/amend.h custom/query.h custom/query_async.h)
endif ()

set_target_properties(dew_python PROPERTIES OUTPUT_NAME "_dew")
target_include_directories(dew_python PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dew_python PRIVATE dew_converter dew_render dew_access dew_core)

# Build the extension into a `dew/` directory next to a copy of the package's
# __init__.py, so the build tree imports exactly the way the wheel does
# (`import dew` -> package -> ._dew) and the tests exercise that path.
set_target_properties(dew_python PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/dew")
# A multi-config generator (MSVC, Xcode) appends the config name to LIBRARY_OUTPUT_DIRECTORY, which
# puts the extension in dew/Release/ while __init__.py stages to dew/ -- so `import dew` from the
# build tree fails on exactly the generators that need it most. Pinning the per-config property
# defeats the suffix; it is a no-op for single-config generators, where the list is empty.
foreach(_cfg IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER "${_cfg}" _cfg_upper)
    set_target_properties(dew_python PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${_cfg_upper} "${CMAKE_CURRENT_BINARY_DIR}/dew")
endforeach()
# Tracked as a real dependency rather than a POST_BUILD step: POST_BUILD only fires when the
# extension itself relinks, so editing only __init__.py used to leave a stale copy in the build tree
# and the tests would import the old package.
# Every .py in the package is staged, not just __init__ -- dew/aio.py is part of the public surface.
set(_dew_py_modules __init__.py aio.py)
set(_dew_py_staged)
foreach(_mod IN LISTS _dew_py_modules)
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/dew/${_mod}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/dew"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/dew/${_mod}"
                "${CMAKE_CURRENT_BINARY_DIR}/dew/${_mod}"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/dew/${_mod}"
        COMMENT "staging dew/${_mod} next to the extension")
    list(APPEND _dew_py_staged "${CMAKE_CURRENT_BINARY_DIR}/dew/${_mod}")
endforeach()
add_custom_target(dew_python_package ALL DEPENDS ${_dew_py_staged})

add_dependencies(dew_python_package dew_python)

if (DEW_PYTHON_REQUIRE_STABLE_ABI)
    # Ground truth after the fact: nanobind links the -abi3 variant of its
    # runtime and uses the abi3 suffix only on a real limited-API build.
    if (NOT TARGET nanobind-static-abi3)
        message(FATAL_ERROR "nanobind declined the stable-ABI build (no nanobind-static-abi3 target)")
    endif ()
    get_target_property(_dew_suffix dew_python SUFFIX)
    if (NOT _dew_suffix STREQUAL "${NB_SUFFIX_S}")
        message(FATAL_ERROR "extension suffix '${_dew_suffix}' is not the stable-ABI suffix "
                            "'${NB_SUFFIX_S}' -- the wheel would be tagged abi3 but be version-locked")
    endif ()
    message(STATUS "dew python: stable-ABI (abi3) build, suffix ${_dew_suffix}")
endif ()

if (APPLE)
    # belt-and-braces: CMake's automatic build rpath already points at the
    # dew_* build dirs; this helps if the module is copied next to the dylibs
    set_target_properties(dew_python PROPERTIES BUILD_RPATH "@loader_path")
endif ()
if (WIN32)
    # dew_access too: dew_python links it (see target_link_libraries above), and without it the
    # extension cannot load from the build tree. The wheel install list already had it right.
    copy_dll_for_target(dew_python dew_render dew_converter dew_access dew_core)
endif ()

# ---------------------------------------------------------------------------
# Wheel installation (scikit-build-core only; ordinary builds install nothing)
#
# Layout inside site-packages:
#   dew/__init__.py, dew/_dew.<abi3>.so       the package + extension
#   dew/lib/libdew_{common,render,converter}  the shared libraries
#   dew/include/dew/**/*.h{,pp}               public C headers + the C++ await wrappers
#   dew/cmake/dewConfig.cmake                 find_package(dew) for consumers
#   <env>/bin/dew                             the CLI (statically linked)
# ---------------------------------------------------------------------------
if (SKBUILD)
    set(_dew_pkg "${SKBUILD_PLATLIB_DIR}/dew")

    # The extension loads its sibling libraries from dew/lib.
    if (APPLE)
        set_target_properties(dew_python PROPERTIES INSTALL_RPATH "@loader_path/lib")
    elseif (UNIX)
        set_target_properties(dew_python PROPERTIES INSTALL_RPATH "$ORIGIN/lib")
    endif ()

    install(TARGETS dew_python LIBRARY DESTINATION "${_dew_pkg}" COMPONENT dew_python)

    # Windows resolves a module's dependencies from the directory containing it,
    # so the DLLs go beside the extension rather than in lib/ -- which is also
    # where delvewheel expects to find them when it repairs the wheel. The
    # import libraries stay in lib/ for linking. Elsewhere the rpath set above
    # points at lib/, so the shared libraries live there.
    install(TARGETS dew_core dew_access dew_render dew_converter
            LIBRARY DESTINATION "${_dew_pkg}/lib" COMPONENT dew_runtime
            RUNTIME DESTINATION "${_dew_pkg}" COMPONENT dew_runtime
            ARCHIVE DESTINATION "${_dew_pkg}/lib" COMPONENT dew_devel)

    # Self-contained (it statically links the object libs and every vendored
    # dependency), so it needs no rpath work to sit on PATH.
    if (TARGET dew)
        install(TARGETS dew RUNTIME DESTINATION "${SKBUILD_SCRIPTS_DIR}" COMPONENT dew_cli)
    endif ()

    install(DIRECTORY
                "${CMAKE_SOURCE_DIR}/src/core/dew"
                "${CMAKE_SOURCE_DIR}/src/access/dew"
                "${CMAKE_SOURCE_DIR}/src/render/dew"
                "${CMAKE_SOURCE_DIR}/src/converter/dew"
                # The generated C++ await wrappers travel with the C headers, so a wheel consumer
                # doing find_package(dew) gets `#include <dew/await.hpp>` for free.
                "${CMAKE_SOURCE_DIR}/bindings/cpp/dew"
            DESTINATION "${_dew_pkg}/include" COMPONENT dew_devel
            FILES_MATCHING
                PATTERN "*.h"
                PATTERN "*.hpp"
                # legacy duplicate of dew/core/error.h (same include guard);
                # excluded from the bindgen manifest, keep it out of the wheel too
                PATTERN "dew/error.h" EXCLUDE)

    include(CMakePackageConfigHelpers)
    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/dewConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/dewConfig.cmake"
        INSTALL_DESTINATION "${_dew_pkg}/cmake"
        NO_SET_AND_CHECK_MACRO)
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/dewConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion)
    install(FILES
                "${CMAKE_CURRENT_BINARY_DIR}/dewConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/dewConfigVersion.cmake"
            DESTINATION "${_dew_pkg}/cmake" COMPONENT dew_devel)
endif ()

if (DEW_BUILD_TESTS)
    # Always available (python + libclang are already required to build at all):
    # fails on an unclassifiable declaration, an unlisted public header, or a
    # change to the API-surface counts, so drift cannot pass unnoticed.
    add_test(NAME bindgen_check
        COMMAND ${Python_EXECUTABLE} ${_bindgen_dir}/parse_headers.py
                --source-root ${CMAKE_SOURCE_DIR}/src
                --manifest ${_bindgen_dir}/public_headers.txt
                --check)

    execute_process(
        COMMAND ${Python_EXECUTABLE} -c "import pytest, numpy"
        RESULT_VARIABLE _dew_have_pytest
        OUTPUT_QUIET ERROR_QUIET)
    if (_dew_have_pytest EQUAL 0)
        add_test(NAME python_bindings_smoke
            COMMAND ${Python_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        set_tests_properties(python_bindings_smoke PROPERTIES
            ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}")
        # The inference rules themselves (class grouping, roles, conventions).
        add_test(NAME bindgen_ir
            COMMAND ${Python_EXECUTABLE} -m pytest -q ${_bindgen_dir}/test_ir.py)
    else ()
        message(STATUS "dew python pytest suites disabled: ${Python_EXECUTABLE} lacks pytest and/or numpy "
                       "(bindgen_check still runs)")
    endif ()
endif ()
