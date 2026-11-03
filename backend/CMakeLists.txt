cmake_minimum_required(VERSION 3.30)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

project(dew_server VERSION 0.1.0 DESCRIPTION "Static-file server for dewfall.limilind.com" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (MSVC)
    string(REGEX REPLACE "/GR" "/GR-" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    string(REGEX REPLACE "/EHsc" "/EHs-c-" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_HAS_EXCEPTIONS=0")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -fno-exceptions")
endif ()

# This is only the backend (a prism static-file server) for dewfall.limilind.com. The renderer itself
# is the dewfall library cross-compiled to WebAssembly separately (src/wasm, via Emscripten).
# vio / structify / prism come from cmake-dep, pinned in cmdep-packages.cmake and
# resolved to one copy each in cmdep.lock (`cmdep lock` after changing a pin). To build
# against a local checkout instead: -DCMDEP_SOURCE_DIR_VIO=/path/to/vio (likewise
# _PRISM, _STRUCTIFY).
include(FetchContent)
FetchContent_Declare(cmake-dep
        GIT_REPOSITORY https://github.com/jorgen/cmake-dep.git
        GIT_TAG 756bee3072d1bf67e179b62a9635cdcdef5931e7)
FetchContent_MakeAvailable(cmake-dep)
list(APPEND CMAKE_MODULE_PATH "${cmake-dep_SOURCE_DIR}/cmake")
include(CmDepMain)
CmDepFetch()

# The dependencies build with exceptions and RTTI where their own code needs them; our
# -fno-exceptions/-fno-rtti apply to this project's targets only.
set(_DEW_SERVER_SAVED_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-fno-exceptions" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-fno-rtti" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "/EHs-c-" "/EHsc" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "/GR-" "/GR" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-D_HAS_EXCEPTIONS=0" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
CmDepAddPackage(vio CONFIG PUBLIC_INCLUDE src
        OPTIONS VIO_BUILD_TESTS=OFF VIO_BUILD_EXAMPLES=OFF VIO_BUILD_GUI=OFF VIO_BUILD_SHARED=OFF VIO_INSTALL=OFF)
CmDepAddPackage(structify CONFIG
        OPTIONS STRUCTIFY_OPT_BUILD_TESTS=OFF STRUCTIFY_OPT_BUILD_EXAMPLES=OFF STRUCTIFY_OPT_BUILD_BENCHMARKS=OFF STRUCTIFY_OPT_INSTALL=OFF)
CmDepAddPackage(prism CONFIG OPTIONS PRISM_BUILD_TESTS=OFF PRISM_BUILD_EXAMPLES=OFF)
set(CMAKE_CXX_FLAGS "${_DEW_SERVER_SAVED_CXX_FLAGS}")

# Consume the dependency headers as SYSTEM headers: prism builds with -Werror -Wextra,
# and vio's headers would otherwise trip -Wmissing-field-initializers and friends in our code.
foreach (dep_target vio structify prism)
    set_property(TARGET ${dep_target} PROPERTY
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
            $<TARGET_PROPERTY:${dep_target},INTERFACE_INCLUDE_DIRECTORIES>)
endforeach ()

add_executable(dew_server src/main.cpp)
target_link_libraries(dew_server PRIVATE prism::prism)
