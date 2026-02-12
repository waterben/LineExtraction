#!/bin/bash
# ==============================================================================
# Python Type Stub Generator
# ==============================================================================
# Generates .pyi type stubs for all LineExtraction Python binding modules
# using pybind11-stubgen. Stubs are placed alongside each module's source
# for IDE support (Pylance, Pyright, mypy).
#
# Prerequisites:
#   - Python bindings built: bazel build //libs/...
#   - pybind11-stubgen installed: pip install pybind11-stubgen (in .venv)
#
# Usage:
#   ./tools/scripts/generate_stubs.sh          # Build + generate
#   ./tools/scripts/generate_stubs.sh --no-build  # Skip build step
#
# Output:
#   libs/imgproc/python/le_imgproc.pyi
#   libs/edge/python/le_edge.pyi
#   libs/geometry/python/le_geometry.pyi
#   libs/eval/python/le_eval.pyi
#   libs/lsd/python/le_lsd.pyi
# ==============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
NC='\033[0m'

# Find project root
if [[ -n "${BUILD_WORKSPACE_DIRECTORY}" ]]; then
    PROJECT_ROOT="${BUILD_WORKSPACE_DIRECTORY}"
elif [[ -f "MODULE.bazel" ]]; then
    PROJECT_ROOT="$(pwd)"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

cd "${PROJECT_ROOT}"

echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}        Generating Python Type Stubs (.pyi)${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# --- Check prerequisites ---

# Activate .venv if not already active
if [[ -z "${VIRTUAL_ENV}" ]] && [[ -f ".venv/bin/activate" ]]; then
    source .venv/bin/activate
fi

if ! command -v pybind11-stubgen &>/dev/null; then
    echo -e "${RED}ERROR: pybind11-stubgen is not installed.${NC}"
    echo "Install with: uv pip install pybind11-stubgen"
    exit 1
fi

# --- Build Python modules (unless --no-build) ---

if [[ "$1" != "--no-build" ]]; then
    echo -e "${BLUE}Building Python binding modules...${NC}"
    bazel build \
        //libs/imgproc/python:le_imgproc \
        //libs/edge/python:le_edge \
        //libs/geometry/python:le_geometry \
        //libs/eval/python:le_eval \
        //libs/lsd/python:le_lsd
    echo ""
fi

# --- Set up PYTHONPATH to find built .so files ---

PYTHONPATH=""
for lib in imgproc edge geometry eval lsd; do
    PYTHONPATH="${PROJECT_ROOT}/bazel-bin/libs/${lib}/python:${PYTHONPATH}"
done
export PYTHONPATH

# --- Module definitions (generation order respects dependencies) ---
# le_imgproc, le_geometry, le_eval: no Python cross-module deps
# le_edge: imports le_imgproc at load time
# le_lsd: imports le_imgproc, le_edge, le_geometry at load time

MODULES=(
    "le_imgproc:libs/imgproc/python"
    "le_geometry:libs/geometry/python"
    "le_eval:libs/eval/python"
    "le_edge:libs/edge/python"
    "le_lsd:libs/lsd/python"
)

# --- Common stubgen flags ---
# --numpy-array-remove-parameters: Simplify numpy.ndarray[...] → numpy.ndarray
#   (pybind11 emits complex ndarray type params that confuse type checkers)
# --ignore-invalid-expressions: Suppress errors from C++ types leaking through
#   (e.g. lsfm::Matx<float, 1> in end_points default args)

STUBGEN_FLAGS=(
    --numpy-array-remove-parameters
    --ignore-invalid-expressions '.*'
)

# --- Generate stubs ---

TMPDIR=$(mktemp -d)
trap 'rm -rf "${TMPDIR}"' EXIT

FAILED=0
for entry in "${MODULES[@]}"; do
    MODULE="${entry%%:*}"
    TARGET_DIR="${entry##*:}"

    echo -n "  Generating ${MODULE}.pyi ... "

    if pybind11-stubgen "${MODULE}" -o "${TMPDIR}" "${STUBGEN_FLAGS[@]}" 2>/dev/null; then
        STUB_FILE="${TMPDIR}/${MODULE}.pyi"
        if [[ -f "${STUB_FILE}" ]]; then
            cp "${STUB_FILE}" "${PROJECT_ROOT}/${TARGET_DIR}/${MODULE}.pyi"
            LINES=$(wc -l < "${STUB_FILE}")
            echo -e "${GREEN}OK${NC} (${LINES} lines)"
        else
            echo -e "${RED}FAIL${NC} (no output file)"
            FAILED=$((FAILED + 1))
        fi
    else
        echo -e "${RED}FAIL${NC} (stubgen error)"
        FAILED=$((FAILED + 1))
    fi
done

echo ""
if [[ ${FAILED} -eq 0 ]]; then
    echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}All type stubs generated successfully!${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
    echo ""
    echo "Generated files:"
    for entry in "${MODULES[@]}"; do
        MODULE="${entry%%:*}"
        TARGET_DIR="${entry##*:}"
        echo "  ${TARGET_DIR}/${MODULE}.pyi"
    done
else
    echo -e "${RED}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${RED}${FAILED} module(s) failed. Check errors above.${NC}"
    echo -e "${RED}═══════════════════════════════════════════════════════════════${NC}"
    exit 1
fi
