#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_dir="$test_dir/../src/MainApp/aggregation"
build_dir="${TMPDIR:-/tmp}/msap1-r5c1-aggregation-tests.$$"

cleanup()
{
	rm -rf -- "$build_dir"
}
trap cleanup EXIT HUP INT TERM

mkdir -p -- "$build_dir"
"${CXX:-c++}" -std=c++20 -Wall -Wextra -Wpedantic -Werror \
	-I"$source_dir" \
	"$test_dir/aggregation_shadow_tests.cpp" \
	"$source_dir/aggregation_frame_decoder.cpp" \
	"$source_dir/aggregation_frame_ring.cpp" \
	"$source_dir/aggregation_health.cpp" \
	"$source_dir/crc32c.cpp" \
	-o "$build_dir/aggregation_shadow_tests"

"$build_dir/aggregation_shadow_tests"
