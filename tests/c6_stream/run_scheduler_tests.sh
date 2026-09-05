#!/usr/bin/env bash
set -euo pipefail
test_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test_binary="$(mktemp "${TMPDIR:-/tmp}/hub75-c6-scheduler.XXXXXX")"
trap 'rm -f -- "$test_binary"' EXIT
component="$test_directory/../../components/hub75"
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer -pthread \
  -I"$test_directory/host" -I"$component/include" \
  "$test_directory/scheduler_test.cpp" \
  "$component/src/platforms/parlio_stream/parlio_stream_dma.cpp" \
  "$component/src/platforms/platform_dma.cpp" -o "$test_binary"
timeout 30 "$test_binary"
