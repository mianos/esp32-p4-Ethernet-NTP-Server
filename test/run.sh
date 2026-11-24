#!/usr/bin/env bash
# Host tests for the firmware's plain-value decisions: everything here is a
# function of values, so nothing needs the board, ESP-IDF, or a network. main/ is
# on the include path and no firmware source is linked, which means a test calls
# the code the firmware calls rather than a copy of it.
#
#     ./test/run.sh
#
# CXX picks the compiler; it needs C++20 for std::span.
set -e
here=$(cd "$(dirname "$0")" && pwd)
cxx=${CXX:-c++}
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

failed=0
for src in "$here"/*_test.cpp; do
    name=$(basename "$src" .cpp)
    "$cxx" -std=c++20 -O2 -Wall -Wextra -Werror -I "$here/../main" "$src" -o "$out/$name"
    "$out/$name" || failed=1
done
exit $failed
