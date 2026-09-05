#!/usr/bin/env bash
set -euo pipefail
test_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
test_binary="$(mktemp "${TMPDIR:-/tmp}/hub75-c6-encoder.XXXXXX")"
trap 'rm -f -- "$test_binary"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -pedantic -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$test_directory/encoder_test.cpp" -o "$test_binary"
"$test_binary"
