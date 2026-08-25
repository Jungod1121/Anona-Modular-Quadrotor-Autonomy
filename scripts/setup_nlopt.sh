#!/usr/bin/env bash
# Build vendored NLopt into third_party/nlopt_install — the prefix
# fp_bspline_opt's find_package(NLopt) searches first. Idempotent: skips
# when the library is already installed there.
set -eo pipefail

cd "$(dirname "$0")/.."

PREFIX="$(pwd)/third_party/nlopt_install"
if [ -f "${PREFIX}/lib/libnlopt.so" ] || [ -f "${PREFIX}/lib64/libnlopt.so" ] ||
   [ -f "${PREFIX}/lib/libnlopt.a" ]; then
  echo "==> nlopt_install already present, skipping"
  exit 0
fi

if [ ! -f "$(pwd)/third_party/nlopt/CMakeLists.txt" ]; then
  echo "ERROR: third_party/nlopt source missing" >&2
  exit 1
fi

echo "==> configuring vendored nlopt -> ${PREFIX}"
cmake -S third_party/nlopt -B third_party/nlopt/build \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DNLOPT_GUILE=OFF -DNLOPT_MATLAB=OFF -DNLOPT_OCTAVE=OFF \
  -DNLOPT_PYTHON=OFF -DNLOPT_FORTRAN=OFF -DNLOPT_TESTS=OFF \
  -DNLOPT_SWIG=OFF

echo "==> building + installing nlopt"
cmake --build third_party/nlopt/build -j"$(nproc)"
cmake --install third_party/nlopt/build
