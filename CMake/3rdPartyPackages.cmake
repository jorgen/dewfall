# 3rdparty package definitions.
# Included by CmDepFetch and CmDepFetchDependencies.
# Caller must define CmDepFetchPackage(name version url url_hash) before including this file.

# One fmt for every target. This used to be split -- 10.1.1 natively, 11.2.0 under Emscripten,
# because fmt 10.x hits a consteval error in format-inl.h with emscripten's clang. 11.2.0 fixed that
# but still failed to build the fmt LIBRARY target for wasm: fmt/format.h's malloc/free allocator
# had no <stdlib.h>, which only surfaces where the standard headers do not transitively pull it in
# (emscripten's libc++). 12.2.0 includes it unconditionally, so a single pin now compiles both.
CmDepFetchPackage(fmt 12.2.0 https://github.com/fmtlib/fmt/archive/12.2.0.tar.gz SHA256=8b852bb5aa6e7d8564f9e81394055395dd1d1936d38dfd3a17792a02bebd7af0)
CmDepFetchPackage(sdl 3.1.6 https://github.com/libsdl-org/SDL/archive/refs/tags/preview-3.1.6.tar.gz SHA256=5da5e265c150b954d007bf1465b155d9df1d0d52f10115a49bb918dc8fe2826a)
CmDepFetchPackage(glm 1.0.1 https://github.com/g-truc/glm/archive/1.0.1.tar.gz SHA256=9f3174561fd26904b23f0db5e560971cbf9b3cbda0b280f04d5c379d03bf234c)
CmDepFetchPackage(doctest 2.4.12 https://github.com/doctest/doctest/archive/v2.4.12.tar.gz SHA256=73381c7aa4dee704bd935609668cf41880ea7f19fa0504a200e13b74999c2d70)
CmDepFetchPackage(cmakerc 952ff https://github.com/vector-of-bool/cmrc/archive/952ffddba731fc110bd50409e8d2b8a06abbd237.zip SHA256=b199e7481dda667cd1b1936c9acb64e496ebc3c5ad90b381ba8d0f361c80638d)
CmDepFetchPackage(imgui 1.91.6 https://github.com/ocornut/imgui/archive/v1.91.6.tar.gz SHA256=c5fbc5dcab1d46064001c3b84d7a88812985cde7e0e9ced03f5677bec1ba502a)
CmDepFetchPackage(vio 9c2c905 https://github.com/jorgen/vio/archive/9c2c9052d65d255d1a481c9c964066dc0a348b39.tar.gz SHA256=5e90b82745aff9cca28ff33f73c8f09df2fdd2ffbb798bf620ce47b4e577afa9)
CmDepFetchPackage(laszip 3.5.0 https://github.com/LASzip/LASzip/archive/refs/tags/3.5.0.tar.gz SHA256=6e9baac8689dfd2e1502ceafabb20c62b6cd572744d240fb755503fd57c2a6af)
CmDepFetchPackage(libmorton 0.2.10 https://github.com/Forceflow/libmorton/archive/v0.2.10.tar.gz SHA256=5f4a8dbf350222b0bec35ad1bee6b76fa22e16ea364e1e80ca47d8f5a7210d6f)
CmDepFetchPackage(argh 431bf323ac https://github.com/adishavit/argh/archive/431bf323acd3bb805483491ff277dac5d85772ad.tar.gz SHA256=af6579d9143b9e2445354796645648c2bbbecdbe3705ba825221861a95f936ea)
CmDepFetchPackage(unordered_dense 4.1.2 https://github.com/martinus/unordered_dense/archive/refs/tags/v4.1.2.tar.gz SHA256=300410dbcd32800f83b2113dfecbdfe8cd256caa4cfeb117d646021d6e3209ae)
CmDepFetchPackage(zstd 1.5.7 https://github.com/facebook/zstd/archive/refs/tags/v1.5.7.tar.gz SHA256=37d7284556b20954e56e1ca85b80226768902e2edabd3b649e9e72c0c9012ee3)
CmDepFetchFile(stbimage b42009b https://raw.githubusercontent.com/nothings/stb/8b5f1f37b5b75829fc72d38e7b5d4bcbf8a26d55/stb_image.h stb_image.h SHA256=91f435e0fc6a620018b878b9859c74dff60d28046f87e649191ad6f35a98c722)

# examples/ford only: the Ford scans are MATLAB v5 files whose payload is a single zlib (RFC1950)
# stream, and nothing else in the tree can inflate one -- zstd's zlibWrapper is an API shim over real
# zlib, not an implementation. Fetched only when examples are built, which also keeps it out of the
# wasm build (EMSCRIPTEN forces DEW_BUILD_EXAMPLES off above CmDepFetch).
if (DEW_BUILD_EXAMPLES)
    CmDepFetchPackage(zlib_ng 2.3.3 https://github.com/zlib-ng/zlib-ng/archive/refs/tags/2.3.3.tar.gz SHA256=f9c65aa9c852eb8255b636fd9f07ce1c406f061ec19a2e7d508b318ca0c907d1)
endif ()

# Python bindings only. The nanobind GitHub tarball does not contain the
# ext/robin_map submodule, so robin-map is fetched separately and exposed as
# the tsl::robin_map target before nanobind is added (see bindings/python).
if (DEW_BUILD_PYTHON)
    CmDepFetchPackage(nanobind 2.13.0 https://github.com/wjakob/nanobind/archive/refs/tags/v2.13.0.tar.gz SHA256=cb25a582ccade4b6067bc73c78b84ad9dbd0bbe0e537320711d18015ccafc4ef)
    CmDepFetchPackage(robin_map 1.4.1 https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.1.tar.gz SHA256=0e3f53a377fdcdc5f9fed7a4c0d4f99e82bbb64175233bd13427fef9a771f4a1)
endif ()
