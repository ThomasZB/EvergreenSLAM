#!/usr/bin/env bash
# ThreadSanitizer build and run for the lifelong tests -- the T9 CI gate.
#
# Intended home is the Linux dev container:
#   docker compose run --rm core-tsan
# Direct use (Linux or macOS with a TSan-capable toolchain):
#   tools/run_tsan.sh [source_dir] [build_dir]
set -euo pipefail

SRC=${1:-$(cd "$(dirname "$0")/.." && pwd)}
OUT=${2:-"$SRC/build-tsan"}

# OpenMP's runtime is not TSan-instrumented and reports false positives inside itself; the
# backend's threaded paths use no OpenMP (only the frontend's RTCSM does), so pin it to one
# thread rather than suppressing reports that could hide real races.
export OMP_NUM_THREADS=1
# With TZ unset, glibc's tzset frees and re-strdups its TZ copy on every mktime (glog calls it
# per log line) under a libc-internal lock TSan cannot see: two threads logging at once report
# a false race. Any value that names a zone short-circuits that path.
export TZ=${TZ:-UTC}
# Fail the run on the first report. Death tests fork; forked children re-exec under the
# threadsafe style (set below), which TSan is fine with.
export TSAN_OPTIONS="halt_on_error=1 second_deadlock_stack=1 ${TSAN_OPTIONS:-}"
export GTEST_DEATH_TEST_STYLE=threadsafe

cmake -S "$SRC/core" -B "$OUT" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DEVERGREENSLAM_BUILD_TESTS=ON \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread"
cmake --build "$OUT" -j8
# Everything under lifelong/ (queue, pool, backend, the end-to-end records included).
ctest --test-dir "$OUT" -R '^lifelong' --output-on-failure --timeout 3000
