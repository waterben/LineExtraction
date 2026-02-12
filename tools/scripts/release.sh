#!/usr/bin/env bash
# ==============================================================================
# Release Script
# ==============================================================================
# Builds the lsfm Python wheel, updates version strings across the project,
# and creates a GitHub release with the wheel as a downloadable artifact.
#
# Versioning follows the date-based scheme: YYYY.MM.DD.COUNTER
# Default: today's date with counter 0 (e.g. 2026.02.12.0).
#
# Usage:
#   ./tools/scripts/release.sh                 # Use today's date, counter 0
#   ./tools/scripts/release.sh 2026.02.12.1    # Explicit version
#   ./tools/scripts/release.sh --draft         # Draft release (today's date)
#   ./tools/scripts/release.sh 2026.02.12.0 --draft
#   ./tools/scripts/release.sh 2026.02.12.0 --prerelease
#
# Requires:
#   - bazel  (build the wheel)
#   - gh     (GitHub CLI, authenticated via ``gh auth login``)
#   - sed    (update version strings)
# ==============================================================================

set -euo pipefail

# --- Colors ------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[0;33m'
NC='\033[0m'

# --- Find project root -------------------------------------------------------
if [[ -f "MODULE.bazel" ]]; then
    PROJECT_ROOT="$(pwd)"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi
cd "${PROJECT_ROOT}"

# --- Parse arguments ----------------------------------------------------------
# First positional arg matching version pattern is the version; rest are gh flags.
VERSION=""
GH_FLAGS=()
for arg in "$@"; do
    if [[ -z "${VERSION}" && "${arg}" =~ ^[0-9]{4}\.[0-9]{2}\.[0-9]{2}\.[0-9]+$ ]]; then
        VERSION="${arg}"
    else
        GH_FLAGS+=("${arg}")
    fi
done

# Default: today's date with counter 0
if [[ -z "${VERSION}" ]]; then
    VERSION="$(date +%Y.%m.%d).0"
fi

TAG="v${VERSION}"

echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Release ${TAG}${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# --- Preflight checks --------------------------------------------------------
MISSING=()
for cmd in bazel gh sed; do
    if ! command -v "${cmd}" &>/dev/null; then
        MISSING+=("${cmd}")
    fi
done
if [[ ${#MISSING[@]} -gt 0 ]]; then
    echo -e "${RED}Error: Missing required tools: ${MISSING[*]}${NC}" >&2
    exit 1
fi

if ! gh auth status &>/dev/null; then
    echo -e "${RED}Error: GitHub CLI not authenticated. Run 'gh auth login' first.${NC}" >&2
    exit 1
fi

# Warn about uncommitted changes
if [[ -n "$(git status --porcelain)" ]]; then
    echo -e "${YELLOW}Warning: Working tree has uncommitted changes.${NC}"
    echo ""
fi

# --- Update version strings ---------------------------------------------------
echo -e "${BLUE}Updating version to ${VERSION} ...${NC}"

# MODULE.bazel: version = "..."
sed -i "s/version = \"[^\"]*\"/version = \"${VERSION}\"/" MODULE.bazel
echo "  MODULE.bazel"

# python/BUILD.bazel: version = "..." (in py_wheel)
sed -i "s/^\(    version = \)\"[^\"]*\"/\1\"${VERSION}\"/" python/BUILD.bazel
echo "  python/BUILD.bazel"

# python/lsfm/__init__.py: __version__ = "..."
sed -i "s/^__version__ = \"[^\"]*\"/__version__ = \"${VERSION}\"/" python/lsfm/__init__.py
echo "  python/lsfm/__init__.py"

echo ""

# --- Build wheel --------------------------------------------------------------
WHEEL_TARGET="//python:lsfm_wheel"
DIST_DIR="dist"

echo -e "${BLUE}Building wheel ...${NC}"
bazel build "${WHEEL_TARGET}"

rm -rf "${DIST_DIR}"
mkdir -p "${DIST_DIR}"
cp bazel-bin/python/lsfm-*.whl "${DIST_DIR}/"

WHEEL=$(ls "${DIST_DIR}"/lsfm-*.whl)
echo -e "  ${GREEN}${WHEEL}${NC}"
echo "  Size: $(du -h "${WHEEL}" | cut -f1)"
echo ""

# --- Create GitHub release ----------------------------------------------------
echo -e "${BLUE}Creating GitHub release ${TAG} ...${NC}"
gh release create "${TAG}" "${WHEEL}" \
    --title "${TAG}" \
    --generate-notes \
    "${GH_FLAGS[@]+"${GH_FLAGS[@]}"}"

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Release ${TAG} created successfully!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo "  Install from file:"
echo "      pip install ${WHEEL}"
echo ""
echo "  Install from GitHub release:"
echo "      gh release download ${TAG} --pattern '*.whl' --dir /tmp"
echo "      pip install /tmp/lsfm-*.whl"
