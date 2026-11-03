# syntax=docker/dockerfile:1
#
# Multi-stage build for dewfall (dewfall.limilind.com): one image serving the built React SPA, which loads
# the WebGL2 point-cloud renderer compiled to WebAssembly. The app is entirely client-side — the point
# cloud is streamed straight from S3 by the WASM's own fetch client — so the server only serves static
# files. Build context is the repo root:
#   docker build -f deploy/Dockerfile -t dewfall-web .

# ---------- WASM (C++ renderer -> WebAssembly via Emscripten) ----------
# Pin the emsdk version the renderer is developed against (keep in sync with deploy.yml).
FROM emscripten/emsdk:6.0.3 AS wasm
WORKDIR /src
# The emsdk image ships CMake 3.28; the dewfall build requires >= 3.30, so install a newer CMake (pip puts it
# in /usr/local/bin, ahead of the image's cmake on PATH). Ninja comes from the same place: without it emcmake
# falls back to Unix Makefiles and the build runs single-threaded.
RUN pip3 install --break-system-packages --no-cache-dir cmake ninja

# Only what the wasm target actually compiles, instead of the whole context. The point is layer reuse: with
# `COPY . ./` any change anywhere — backend/, tools/, tests/, and above all examples/renderer/web/ — busts this
# layer and re-runs a multi-minute Emscripten build that could not possibly be affected by it. The SPA lives in
# examples/renderer/web/ and is deliberately excluded here; it is copied into the `web` stage instead.
#
# This set is exact, not approximate — it was validated by building both wasm targets from a tree pruned to
# precisely these paths. src/wasm/CMakeLists.txt reaches into examples/renderer for renderer_wasm.cpp,
# gl_renderer.cpp and the cmrc-embedded shaders, and CMakeLists.txt adds bindings/cpp unconditionally.
COPY CMakeLists.txt CMakePresets.json ./
COPY CMake/ ./CMake/
COPY src/ ./src/
COPY bindings/cpp/ ./bindings/cpp/
COPY examples/renderer/shaders/ ./examples/renderer/shaders/
COPY examples/renderer/include/ ./examples/renderer/include/
COPY examples/renderer/renderer_wasm.cpp examples/renderer/gl_renderer.cpp examples/renderer/gl_renderer.h ./examples/renderer/

# Configure the dewfall project under Emscripten and build only the WebGL2 renderer module. (The build-preset
# 'emscripten' defaults to dew_data_wasm, so the target is named explicitly.) Fetches the pinned CmDep
# dependencies (vio, zstd, fmt, glm, ...) during configure.
RUN emcmake cmake -S . -B /wasm-build -G Ninja -DCMAKE_BUILD_TYPE=Release \
 && cmake --build /wasm-build --target dew_render_wasm dew_decode_worker \
 && test -f /wasm-build/src/wasm/dew_render.mjs \
 && test -f /wasm-build/src/wasm/dew_render.wasm \
 && test -f /wasm-build/src/wasm/dew_decode_worker.mjs \
 && test -f /wasm-build/src/wasm/dew_decode_worker.wasm

# ---------- SPA (React + Vite) ----------
FROM node:20-slim AS web
WORKDIR /web
COPY examples/renderer/web/package.json examples/renderer/web/package-lock.json ./
RUN npm ci
COPY examples/renderer/web/ ./
# The renderer bundle + decode-worker module from the wasm stage; copy-wasm.mjs (npm prebuild) pulls
# them from DEW_WASM_DIR. The worker enables off-main-thread decode (the app probes for it via HEAD).
COPY --from=wasm /wasm-build/src/wasm/dew_render.mjs /wasm-build/src/wasm/dew_render.wasm \
                 /wasm-build/src/wasm/dew_decode_worker.mjs /wasm-build/src/wasm/dew_decode_worker.wasm /wasm-artifacts/
ENV DEW_WASM_DIR=/wasm-artifacts
RUN npm run build   # copy-wasm -> tsc --noEmit && vite build -> /web/dist

# ---------- backend (prism static-file server) ----------
FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive CC=gcc-14 CXX=g++-14
RUN apt-get update && apt-get install -y --no-install-recommends \
      g++-14 ninja-build git ca-certificates python3-pip \
 && pip3 install --break-system-packages cmake \
 && rm -rf /var/lib/apt/lists/*

COPY backend/ /src/backend/
# Dependencies (vio, structify, prism) are pinned in backend/cmdep-packages.cmake and fetched by
# cmake-dep; CMDEP_LOCKED fails the build if backend/cmdep.lock is out of date.
RUN --mount=type=cache,target=/root/.cache/cmake-dep \
    cmake -S /src/backend -B /build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMDEP_LOCKED=ON \
      -DCMAKE_EXE_LINKER_FLAGS="-static-libstdc++ -static-libgcc" \
 && cmake --build /build --target dew_server \
 && (ldd /build/dew_server || true)

# ---------- runtime ----------
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates tzdata \
 && rm -rf /var/lib/apt/lists/*
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt \
    DEW_DIST=/app/dist \
    DEW_HOST=0.0.0.0 \
    DEW_PORT=8080
COPY --from=build /build/dew_server /usr/local/bin/dew_server
COPY --from=web   /web/dist            /app/dist
RUN useradd --system --uid 10001 --user-group app
USER app
EXPOSE 8080
ENTRYPOINT ["/usr/local/bin/dew_server"]
