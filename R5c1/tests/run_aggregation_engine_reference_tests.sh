#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_dir="$test_dir/../src/MainApp/aggregation"
pl_hls_dir="$test_dir/../../../MSAP1_PL/SourceData/HLS_DesignFile"
vitis_root="${XILINX_VITIS:-/opt/Xilinx/2025.2/Vitis}"
build_dir=$(mktemp -d \
	"${TMPDIR:-/tmp}/msap1-r5c1-aggregation-reference-tests.XXXXXXXX")

cleanup()
{
	rm -rf -- "$build_dir"
}
trap cleanup EXIT HUP INT TERM

cxx=${CXX:-c++}
common_flags="-std=c++20 -O2 -Wall -Wextra -Wpedantic \
	-Wno-unknown-pragmas \
	-Wno-extra \
	-Wno-volatile \
	-Wno-unused-function \
	-Wno-unused-variable \
	-Wno-deprecated-copy \
	-Wno-type-limits \
	-Wno-deprecated-enum-enum-conversion \
	-Wno-maybe-uninitialized \
	-Wno-uninitialized \
	-Wno-comment \
	-Wno-pedantic \
	-Wno-unused-label \
	-Wno-sign-compare \
	-I$vitis_root/include \
	-I$source_dir \
	-I$pl_hls_dir/common/include"

if [ "${MNC_REQUIRE_IEC_UTC_OVERLAP:-0}" = "1" ]; then
	common_flags="$common_flags -DMNC_REQUIRE_IEC_UTC_OVERLAP=1"
fi
if [ "${MNC_REQUIRE_M15_INVALIDATION_MATRIX:-0}" = "1" ]; then
	common_flags="$common_flags -DMNC_REQUIRE_M15_INVALIDATION_MATRIX=1"
fi

gmp_library=$($cxx -print-file-name=libgmp.so)
if [ "$gmp_library" = "libgmp.so" ]; then
	multiarch=$($cxx -print-multiarch 2>/dev/null || true)
	for candidate in \
		"/lib/$multiarch/libgmp.so.10" \
		"/usr/lib/$multiarch/libgmp.so.10"
	do
		if [ -f "$candidate" ]; then
			gmp_library=$candidate
			break
		fi
	done
fi

if [ ! -e "$gmp_library" ]; then
	echo "Unable to locate a host GMP library for the Vitis ap_int simulation" >&2
	exit 1
fi

build_variant()
{
	variant=$1
	preview_define=$2
	# shellcheck disable=SC2086
	"$cxx" $common_flags $preview_define \
		-c "$test_dir/aggregation_engine_reference_tests.cpp" \
		-o "$build_dir/reference-$variant.o"
	# shellcheck disable=SC2086
	"$cxx" $common_flags $preview_define \
		-c "$source_dir/aggregation_engine.cpp" \
		-o "$build_dir/engine-$variant.o"
	"$cxx" "$build_dir/reference-$variant.o" \
		"$build_dir/engine-$variant.o" "$gmp_library" \
		-o "$build_dir/reference-$variant"
}

build_variant previews-on "-DMNC_AGGREGATION_ENABLE_OPEN_PREVIEWS=1"
build_variant previews-off "-DMNC_AGGREGATION_ENABLE_OPEN_PREVIEWS=0"

MNC_COMPLETED_RECORD_TRACE="$build_dir/previews-on.bin" \
	"$build_dir/reference-previews-on"
MNC_COMPLETED_RECORD_TRACE="$build_dir/previews-off.bin" \
	"$build_dir/reference-previews-off"

cmp "$build_dir/previews-on.bin" "$build_dir/previews-off.bin"
echo "PASS: completed records are byte-identical with previews enabled and disabled"
