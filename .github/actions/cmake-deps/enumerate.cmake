# Enumerate the dependency set a cmake-dep build fetches, without fetching anything.
#
#   cmake [-DDEW_BUILD_PYTHON=ON] \
#         -DCMDEP_PACKAGES_FILE=/abs/path/CMake/3rdPartyPackages.cmake \
#         [-DCMDEP_LOCK_FILE=/abs/path/CMake/cmdep.lock] \
#         -DCMDEP_ENUM_OUT=/abs/path/manifest.txt \
#         -DCMDEP_PROJECT=dewfall \
#         -P enumerate.cmake
#
# One record per dependency: the root packages file's declarations in declaration order, then the
# transitive packages from the lockfile (vio's boringssl, ada, ...), which a configure fetches too.
#
#   KIND|name|version|entry|ALGO|hex|url|payload
#
#   KIND    PACKAGE (archive or git commit, extracted), FILE (single downloaded file),
#           or SYSTEM (USE_SYSTEM is on, so nothing is fetched)
#   entry   the cache entry cmake-dep creates: <CMDEP_DIR>/<name>-<version>-<hash12>
#   payload "" for PACKAGE/SYSTEM, the destination filename for FILE
#
# The archive behind each record is <CMDEP_DIR>/archives/<algo>/<hex>, which is what CI caches.
#
# The packages file is evaluated by the real interpreter, so `if (DEW_BUILD_PYTHON)` blocks are
# handled by definition. If it grows a cmake-dep command this shim does not model, CMake fails
# loudly here instead of silently under-reporting the set.
#
# Override resolution mirrors cmake-dep's (<PREFIX>_<DEP>_VERSION / _URL / _<ALGO> and
# <PREFIX>_USE_SYSTEM_<DEP>), so a build configured with -D overrides enumerates what it fetches.

cmake_minimum_required(VERSION 3.20)

if (NOT DEFINED CMDEP_PACKAGES_FILE)
    message(FATAL_ERROR "enumerate: -DCMDEP_PACKAGES_FILE=<path> is required")
endif ()
get_filename_component(CMDEP_PACKAGES_FILE "${CMDEP_PACKAGES_FILE}" ABSOLUTE)
if (NOT EXISTS "${CMDEP_PACKAGES_FILE}")
    message(FATAL_ERROR "enumerate: packages file not found: ${CMDEP_PACKAGES_FILE}")
endif ()
if (NOT DEFINED CMDEP_LOCK_FILE OR CMDEP_LOCK_FILE STREQUAL "")
    get_filename_component(_pkg_dir "${CMDEP_PACKAGES_FILE}" DIRECTORY)
    set(CMDEP_LOCK_FILE "${_pkg_dir}/cmdep.lock")
endif ()

# Required rather than defaulted: the knob prefix must be the consuming project's project() name
# (dewfall -> DEWFALL_<DEP>_VERSION and friends), or a -D override moves the build without moving
# the manifest.
if (NOT DEFINED CMDEP_PROJECT OR CMDEP_PROJECT STREQUAL "")
    message(FATAL_ERROR "enumerate: -DCMDEP_PROJECT=<project() name> is required "
                        "(cmake-dep derives its override-knob prefix from it)")
endif ()
string(TOUPPER "${CMDEP_PROJECT}" _prefix)
string(MAKE_C_IDENTIFIER "${_prefix}" _prefix)

set(_records "")
set(_names "")

macro(_emit kind name version algo hex url payload)
    string(TOLOWER "${hex}" _e_hex)
    string(SUBSTRING "${_e_hex}" 0 12 _e_short)
    list(APPEND _records "${kind}|${name}|${version}|${name}-${version}-${_e_short}|${algo}|${_e_hex}|${url}|${payload}")
    list(APPEND _names "${name}")
endmacro()

macro(_resolve name raw_version raw_url raw_hash out_ver out_url out_algo out_hex out_sys)
    string(TOUPPER "${name}" _d)
    string(MAKE_C_IDENTIFIER "${_d}" _d)

    string(REGEX MATCH "^([^=]+)=(.*)$" _m "${raw_hash}")
    if (NOT _m)
        message(FATAL_ERROR "enumerate: hash for '${name}' must be ALGO=HEX, got '${raw_hash}'")
    endif ()
    string(TOUPPER "${CMAKE_MATCH_1}" ${out_algo})
    set(${out_hex} "${CMAKE_MATCH_2}")

    set(${out_ver} "${raw_version}")
    if (DEFINED ${_prefix}_${_d}_VERSION AND NOT "${${_prefix}_${_d}_VERSION}" STREQUAL "")
        set(${out_ver} "${${_prefix}_${_d}_VERSION}")
    endif ()
    set(${out_url} "${raw_url}")
    if (DEFINED ${_prefix}_${_d}_URL AND NOT "${${_prefix}_${_d}_URL}" STREQUAL "")
        set(${out_url} "${${_prefix}_${_d}_URL}")
    endif ()
    if (DEFINED ${_prefix}_${_d}_${${out_algo}} AND NOT "${${_prefix}_${_d}_${${out_algo}}}" STREQUAL "")
        set(${out_hex} "${${_prefix}_${_d}_${${out_algo}}}")
    endif ()

    set(${out_sys} OFF)
    if (${_prefix}_USE_SYSTEM_${_d})
        set(${out_sys} ON)
    endif ()
endmacro()

macro(CmDepFetchPackage name version url url_hash)
    _resolve("${name}" "${version}" "${url}" "${url_hash}" _v _u _a _h _s)
    if (_s)
        _emit(SYSTEM "${name}" "${_v}" "${_a}" "${_h}" "${_u}" "")
    else ()
        _emit(PACKAGE "${name}" "${_v}" "${_a}" "${_h}" "${_u}" "")
    endif ()
endmacro()

# The git archive of a pinned commit, hash-addressed like any other archive. A declaration without
# SHA256= is cloned directly and never lands in the archive store, so there would be nothing to cache.
macro(CmDepFetchGit name version repo commit)
    set(_git_hash "")
    foreach (_arg ${ARGN})
        if (_arg MATCHES "^SHA256=")
            set(_git_hash "${_arg}")
        endif ()
    endforeach ()
    if (_git_hash STREQUAL "")
        message(FATAL_ERROR "enumerate: CmDepFetchGit(${name}) has no SHA256=; pin one so CI can cache it")
    endif ()
    CmDepFetchPackage("${name}" "${version}" "${repo}@${commit}" "${_git_hash}")
endmacro()

macro(CmDepFetchFile name version url destination_name url_hash)
    _resolve("${name}" "${version}" "${url}" "${url_hash}" _v _u _a _h _s)
    _emit(FILE "${name}" "${_v}" "${_a}" "${_h}" "${_u}" "${destination_name}")
endmacro()

include("${CMDEP_PACKAGES_FILE}")

# Transitive packages: everything the lock resolved that the root does not declare itself. A
# configure fetches these up front, before the dependencies that declare them are even added.
macro(CmDepLockFormat version)
endmacro()
macro(CmDepLocked name)
    cmake_parse_arguments(_L "" "VERSION;URL;GIT;COMMIT;HASH;DECLARED_BY;DECLARED_IN" "PATCHES;REQUESTED_BY" ${ARGN})
    if (NOT _L_DECLARED_BY STREQUAL ".")
        list(FIND _names "${name}" _seen)
        if (_seen EQUAL -1)
            if (_L_GIT)
                set(_L_URL "${_L_GIT}@${_L_COMMIT}")
            endif ()
            CmDepFetchPackage("${name}" "${_L_VERSION}" "${_L_URL}" "${_L_HASH}")
        endif ()
    endif ()
endmacro()
if (EXISTS "${CMDEP_LOCK_FILE}")
    include("${CMDEP_LOCK_FILE}")
endif ()

list(LENGTH _records _count)
if (_count EQUAL 0)
    message(FATAL_ERROR "enumerate: parsed 0 dependencies from ${CMDEP_PACKAGES_FILE} -- refusing to report an empty set")
endif ()

set(_text "")
foreach (_r IN LISTS _records)
    string(APPEND _text "${_r}\n")
endforeach ()

if (DEFINED CMDEP_ENUM_OUT AND NOT CMDEP_ENUM_OUT STREQUAL "")
    file(WRITE "${CMDEP_ENUM_OUT}" "${_text}")
else ()
    # message(NOTICE) goes to stderr, so `$(cmake -P ...)` captures nothing without a redirect.
    foreach (_r IN LISTS _records)
        message(NOTICE "${_r}")
    endforeach ()
endif ()
