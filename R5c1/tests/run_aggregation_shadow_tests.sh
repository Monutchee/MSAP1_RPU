#!/bin/sh
set -eu

test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_dir="$test_dir/../src/MainApp/aggregation"
common_dir="$test_dir/../../common/include"
pl_hls_dir="$test_dir/../../../MSAP1_PL/SourceData/HLS_DesignFile"
vitis_root="${XILINX_VITIS:-/opt/Xilinx/2025.2/Vitis}"
build_dir="${TMPDIR:-/tmp}/msap1-r5c1-aggregation-tests.$$"

cleanup()
{
	rm -rf -- "$build_dir"
}
trap cleanup EXIT HUP INT TERM

mkdir -p -- "$build_dir"
cxx=${CXX:-c++}
common_flags="-std=c++20 -O2 -Wall -Wextra -Wpedantic -Werror \
	-Wno-unknown-pragmas \
	-Wno-error=deprecated-copy \
	-Wno-error=type-limits \
	-Wno-error=deprecated-enum-enum-conversion \
	-Wno-error=maybe-uninitialized \
	-Wno-error=uninitialized \
	-Wno-error=comment \
	-Wno-error=pedantic \
	-Wno-error=unused-label \
	-I$source_dir \
	-I$common_dir \
	-I$source_dir/hls_compat \
	-I$pl_hls_dir/common/include \
	-I$vitis_root/include"

# Keep every RPU-owned translation unit warning-clean.  The Vitis ap_int
# implementation is compiled separately because current host GCC releases
# diagnose implementation details inside AMD's headers (deprecated implicit
# copies and unsigned comparisons).  Those diagnostics are unrelated to the
# RPU adapter and must not weaken warning enforcement for our own code.
for source in \
	"$test_dir/aggregation_shadow_tests.cpp" \
	"$source_dir/aggregation_frame_decoder.cpp" \
	"$source_dir/aggregation_frame_ring.cpp" \
	"$source_dir/aggregation_health.cpp" \
	"$source_dir/aggregation_record_ring.cpp" \
	"$source_dir/crc32c.cpp" \
	"$source_dir/flicker_engine.cpp" \
	"$source_dir/flicker_frame_decoder.cpp" \
	"$source_dir/mains_signal_engine.cpp" \
	"$source_dir/mains_signal_frame_decoder.cpp" \
	"$source_dir/harmonic_frame_decoder.cpp" \
	"$source_dir/pq_event_frame_decoder.cpp" \
	"$source_dir/pq_event_lifecycle_engine.cpp" \
	"$source_dir/r5_aggregation_engine.cpp"
do
	object="$build_dir/$(basename "$source" .cpp).o"
	# shellcheck disable=SC2086
	"$cxx" $common_flags -c "$source" -o "$object"
done

# Both arithmetic engines instantiate AMD ap_int operations. Current host GCC
# diagnoses implementation details in those vendor templates; keep the narrow
# exceptions local to the two translation units that actually instantiate
# them rather than weakening warning enforcement for RPU-owned code.
for source in \
	"$source_dir/aggregation_engine.cpp" \
	"$source_dir/energy_demand_engine.cpp" \
	"$source_dir/harmonic_aggregation_engine.cpp"
do
	object="$build_dir/$(basename "$source" .cpp).o"
	# shellcheck disable=SC2086
	"$cxx" $common_flags \
		-Wno-error=extra \
		-Wno-error=sign-compare \
		-c "$source" \
		-o "$object"
done

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

"$cxx" "$build_dir"/*.o "$gmp_library" -o "$build_dir/aggregation_shadow_tests"

"$build_dir/aggregation_shadow_tests"
