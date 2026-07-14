#!/usr/bin/env bash
# Generate amd_platform_info.h (OpenAMP channel config) for both R5 cores.
#
# Runs the Yocto-built lopper openamp assist (--openamp_header_only) over the
# per-domain device trees that gen-machineconf produced from the domain YAML
# (meta-zynqmp-addon/recipes-bsp/domainyaml/openamp-overlay-zynqmp-*.yaml).
# The generated header carries the same macros platform_info.h #ifndef-guards
# (SHARED_MEM_PA, SHARED_MEM_SIZE, SHARED_BUF_OFFSET, IPI_IRQ_VECT_ID,
# POLL_BASE_ADDR, IPI_CHN_BITMASK, ...), derived from the single domain-YAML
# source of truth instead of hand-maintained constants.
#
# Prerequisites:
#   - gen-machineconf has been run (or OPENAMP_DTS_DIR points at its staged DTS)
#   - its esw-conf-native sysroot is available (provides lopper + python)
#   - lopper carries BOTH local fixes from meta-zynqmp-addon/recipes-kernel/lopper
#     (0001 split-mode dual-R5, 0002 header-only dual-R5 + SHARED_MEM_SIZE).
#     If the sysroot assist still has the 0002 bug (2nd core -> silent
#     IndexError, SHARED_MEM_SIZE = buffer address), this script shadow-patches
#     a temp copy so it works before the rebuild:
#       bitbake -c cleansstate lopper-native esw-conf-native
#
# Input/output overrides used by the machine-config build stage:
#   OPENAMP_DTS_DIR   directory containing the per-core generated DTS files
#   OPENAMP_OUT_ROOT  destination root for the two generated headers
# Output by default: <project>/runtime-generated/openamp_gen/psu_cortexr5_{0,1}/amd_platform_info.h
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"   # MSAP1 workspace root
MACHINE="${MACHINE:-msap1}"
DTS_DIR="${OPENAMP_DTS_DIR:-${PROJECT_ROOT}/yocto-build/build/conf/dts/${MACHINE}}"
OUT_ROOT="${OPENAMP_OUT_ROOT:-${PROJECT_ROOT}/runtime-generated/openamp_gen}"

# --- locate the esw-conf-native sysroot (lopper + nativepython3 + dtc) -------
SYSROOT="${LOPPER_SYSROOT:-}"
if [ -z "${SYSROOT}" ]; then
    while IFS= read -r -d '' lopper; do
        [ -x "${lopper}" ] || continue
        SYSROOT="${lopper%/usr/bin/lopper}"
        break
    done < <(find "${PROJECT_ROOT}/yocto-build/build/tmp/work" \
        -path '*/esw-conf-native/*/recipe-sysroot-native/usr/bin/lopper' \
        -type f -print0 2>/dev/null)
fi
if [ -z "${SYSROOT}" ] || [ ! -x "${SYSROOT}/usr/bin/lopper" ]; then
    echo "ERROR: no lopper found. Build esw-conf-native first, or set LOPPER_SYSROOT." >&2
    exit 1
fi
export PATH="${SYSROOT}/usr/bin:${PATH}"
export LOPPER_DTC_FLAGS="-b 0 -@"

# --- shadow-patch the assist if the sysroot copy predates the 0002 fix -------
ASSIST_FILE="$(find "${SYSROOT}/usr/lib" -name openamp_xlnx.py -path '*/lopper/assists/*' -print -quit)"
if [ -z "${ASSIST_FILE}" ]; then
    echo "ERROR: openamp_xlnx.py not found below ${SYSROOT}/usr/lib" >&2
    exit 1
fi
ASSIST_DIR="$(dirname "${ASSIST_FILE}")"
if grep -q "vdev0buffer' in n.name" "${ASSIST_DIR}/openamp_xlnx.py"; then
    echo "NOTE: sysroot lopper lacks the 0002 header-only fix; using a shadow-patched copy."
    echo "      Rebuild for a permanent fix: bitbake -c cleansstate lopper-native esw-conf-native"
    SHADOW="$(mktemp -d)"
    trap 'rm -rf "${SHADOW}"' EXIT
    cp "${ASSIST_DIR}"/openamp.py "${ASSIST_DIR}"/openamp_xlnx.py "${ASSIST_DIR}"/openamp_xlnx_common.py "${SHADOW}/"
    python3 - "${SHADOW}/openamp_xlnx.py" <<'EOF'
import sys
p = sys.argv[1]
src = open(p).read()
old = """    shbuf_sz = hex([n.propval("reg")[1] for n in memory_region_nodes if 'vdev0buffer' in n.name][0])"""
new = """    shbuf_sz = hex(sum([n.propval("reg")[3] for n in vrings]) + [n.propval("reg")[3] for n in memory_region_nodes if 'buffer' in n.name][0])"""
assert src.count(old) == 1, "assist source drifted; update this script/patch 0002"
open(p, 'w').write(src.replace(old, new))
EOF
    export PYTHONPATH="${SHADOW}${PYTHONPATH:+:${PYTHONPATH}}"
    ASSIST_SEARCH="${SHADOW}"
else
    ASSIST_SEARCH="${ASSIST_DIR}"
fi

# --- generate per core --------------------------------------------------------
STATUS=0
for N in 0 1; do
    ESW_MACHINE="psu_cortexr5_${N}"
    DTS="${DTS_DIR}/${MACHINE}-cortexr5-${N}-freertos.dts"
    OUT_DIR="${OUT_ROOT}/${ESW_MACHINE}"
    HDR="${OUT_DIR}/amd_platform_info.h"
    if [ ! -f "${DTS}" ]; then
        echo "ERROR: ${DTS} not found (run gen-machineconf first)" >&2
        STATUS=1
        continue
    fi
    mkdir -p "${OUT_DIR}"
    rm -f "${HDR}"
    ( cd "${OUT_DIR}" && lopper -f --enhanced --permissive -O "${ASSIST_SEARCH}" \
        "${DTS}" -- openamp --openamp_header_only \
        --openamp_output_filename="${HDR}" \
        --openamp_remote="${ESW_MACHINE}" > lopper.log 2>&1 ) || true
    if [ -s "${HDR}" ]; then
        echo "OK  ${HDR}"
        grep -E '#define (IPI_IRQ_VECT_ID|POLL_BASE_ADDR|IPI_CHN_BITMASK|SHARED_MEM_PA|SHARED_MEM_SIZE|SHARED_BUF_OFFSET)' "${HDR}" | sed 's/^/      /'
    else
        echo "FAIL ${ESW_MACHINE}: header not generated -- see ${OUT_DIR}/lopper.log" >&2
        STATUS=1
    fi
done
exit ${STATUS}
