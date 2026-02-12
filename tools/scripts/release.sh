#!/usr/bin/env bash
# ==============================================================================
# Release Script
# ==============================================================================
# Builds the lsfm Python wheel, stamps the release version into source files,
# creates a Git tag, and publishes a GitHub release with the wheel artifact.
# Version strings are restored to "0.0.0.dev0" (PEP 440) after building;
# the release version only lives in the Git tag and the built wheel.
#
# Version scheme: YYYY.MM.DD.COUNTER (e.g. 2026.02.12.0)
# Default: today's date with counter 0.
#
# Usage:
#   ./tools/scripts/release.sh                 # Today's date, counter 0
#   ./tools/scripts/release.sh 2026.02.12.1    # Explicit version
#   ./tools/scripts/release.sh --draft         # Draft release
#   ./tools/scripts/release.sh 2026.02.12.0 --prerelease
#
# Safety checks:
#   - Warns if not on 'main' branch (with confirmation prompt)
#   - Warns if working tree has uncommitted changes (with confirmation prompt)
#   - Prevents releasing if the tag already exists
#   - Warns if the current commit already has a release tag
#
# Requires: bazel, gh (authenticated), git
# ==============================================================================

set -euo pipefail

# --- Configuration -----------------------------------------------------------
WHEEL_TARGET="//python:lsfm_wheel"
DIST_DIR="dist"
MAIN_BRANCH="main"

# --- Helpers -----------------------------------------------------------------
die()     { echo "Error: $1" >&2; exit 1; }
confirm() { read -rp "$1 [y/N] " ans; [[ "${ans}" =~ ^[Yy]$ ]] || exit 0; }

# --- Find project root -------------------------------------------------------
if [[ -f "MODULE.bazel" ]]; then
    PROJECT_ROOT="$(pwd)"
else
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi
cd "${PROJECT_ROOT}"

# --- Parse arguments ----------------------------------------------------------
VERSION=""
GH_FLAGS=()
for arg in "$@"; do
    if [[ -z "${VERSION}" && "${arg}" =~ ^[0-9]{4}\.[0-9]{2}\.[0-9]{2}\.[0-9]+$ ]]; then
        VERSION="${arg}"
    else
        GH_FLAGS+=("${arg}")
    fi
done
VERSION="${VERSION:-$(date +%Y.%m.%d).0}"
TAG="v${VERSION}"

# --- Preflight checks --------------------------------------------------------

# Required tools
for cmd in bazel gh git; do
    command -v "${cmd}" &>/dev/null || die "'${cmd}' not found. Please install it first."
done

# GitHub CLI authenticated
gh auth status &>/dev/null || die "GitHub CLI not authenticated. Run 'gh auth login' first."

# Version format
[[ "${VERSION}" =~ ^[0-9]{4}\.[0-9]{2}\.[0-9]{2}\.[0-9]+$ ]] \
    || die "Version must follow YYYY.MM.DD.N format (got: ${VERSION})."

# Uncommitted changes
if ! git diff --quiet HEAD 2>/dev/null || ! git diff --cached --quiet HEAD 2>/dev/null; then
    echo "Warning: You have uncommitted changes:"
    git status --short
    echo ""
    confirm "Release anyway?"
fi

# Branch check
CURRENT_BRANCH="$(git branch --show-current)"
if [[ "${CURRENT_BRANCH}" != "${MAIN_BRANCH}" ]]; then
    echo "Warning: You are on branch '${CURRENT_BRANCH}', not '${MAIN_BRANCH}'."
    echo "Releases are normally created from '${MAIN_BRANCH}'."
    echo ""
    confirm "Continue releasing from '${CURRENT_BRANCH}'?"
fi

# Tag already exists (check both local and remote)
if git rev-parse "${TAG}" &>/dev/null \
    || git ls-remote --tags origin "refs/tags/${TAG}" 2>/dev/null | grep -q "${TAG}"; then
    NEXT_COUNTER=$((${VERSION##*.} + 1))
    NEXT_VERSION="${VERSION%.*}.${NEXT_COUNTER}"
    die "Tag '${TAG}' already exists (local or remote). Try: $0 ${NEXT_VERSION}"
fi

# Commit already tagged (duplicate release)
HEAD_SHORT="$(git rev-parse --short HEAD)"
EXISTING_TAG="$(git tag --points-at HEAD 2>/dev/null | head -n1)"
if [[ -n "${EXISTING_TAG}" ]]; then
    echo "Warning: Commit ${HEAD_SHORT} is already tagged as '${EXISTING_TAG}'."
    confirm "Create an additional release for this commit?"
fi

# --- Summary -----------------------------------------------------------------
echo ""
echo "=== Release Summary ==="
echo "  Version: ${VERSION}"
echo "  Tag:     ${TAG}"
echo "  Branch:  ${CURRENT_BRANCH}"
echo "  Commit:  ${HEAD_SHORT} ($(git log -1 --format='%s' HEAD))"
echo "  Flags:   ${GH_FLAGS[*]:-none}"
echo ""
confirm "Proceed?"

# --- Stamp version -----------------------------------------------------------
DEV_VERSION="0.0.0.dev0"

# Use regex patterns so stamping works regardless of current version value.
# restore_dev uses the exact release version (known at that point).
stamp_version() {
    local ver="$1"
    sed -i "s/version = \"[^\"]*\"/version = \"${ver}\"/" MODULE.bazel
    # Match only the indented version line inside py_wheel() — not the comment
    sed -i "s/^\(    version = \)\"[^\"]*\"/\1\"${ver}\"/" python/BUILD.bazel
    sed -i "s/^__version__ = \"[^\"]*\"/__version__ = \"${ver}\"/" python/lsfm/__init__.py
}

verify_stamp() {
    local ver="$1" ok=1
    grep -q "version = \"${ver}\"" MODULE.bazel          || { echo "  FAIL: MODULE.bazel"; ok=0; }
    grep -q "version = \"${ver}\"" python/BUILD.bazel     || { echo "  FAIL: python/BUILD.bazel"; ok=0; }
    grep -q "__version__ = \"${ver}\"" python/lsfm/__init__.py || { echo "  FAIL: python/lsfm/__init__.py"; ok=0; }
    [[ ${ok} -eq 1 ]] || die "Version stamping to '${ver}' failed. Check files above."
}

restore_dev() {
    echo "=== Restoring dev version ==="
    stamp_version "${DEV_VERSION}"
}

echo "=== Stamping version ${VERSION} ==="
stamp_version "${VERSION}"
verify_stamp "${VERSION}"
trap restore_dev EXIT INT TERM
echo "  MODULE.bazel, python/BUILD.bazel, python/lsfm/__init__.py"

# --- Build wheel --------------------------------------------------------------
echo "=== Building wheel ==="
bazel build "${WHEEL_TARGET}"

# --- Restore dev version (via trap, but run eagerly so later steps see dev) ---
trap - EXIT INT TERM
restore_dev

# --- Collect artifacts --------------------------------------------------------
echo "=== Collecting artifacts ==="
rm -rf "${DIST_DIR}"
mkdir -p "${DIST_DIR}"

WHEEL_PATTERN="lsfm-${VERSION}-*.whl"
cp "bazel-bin/python/${WHEEL_PATTERN}" "${DIST_DIR}/" 2>/dev/null \
    || die "No wheel matching '${WHEEL_PATTERN}' found in bazel-bin/python/."

WHEELS=("${DIST_DIR}"/${WHEEL_PATTERN})
if [[ ${#WHEELS[@]} -ne 1 ]]; then
    die "Expected exactly 1 wheel, found ${#WHEELS[@]}: ${WHEELS[*]}"
fi
WHEEL="${WHEELS[0]}"
echo "  ${WHEEL} ($(du -h "${WHEEL}" | cut -f1))"

# --- Create tag and release ---------------------------------------------------
echo "=== Creating tag ${TAG} ==="
git tag -a "${TAG}" -m "Release ${VERSION}" HEAD
git push origin "${TAG}"

echo "=== Creating GitHub release ==="
gh release create "${TAG}" "${WHEEL}" \
    --title "${TAG}" \
    --generate-notes \
    "${GH_FLAGS[@]+"${GH_FLAGS[@]}"}"

echo ""
echo "=== Done ==="
echo "  https://github.com/waterben/LineExtraction/releases/tag/${TAG}"
echo ""
echo "  Install: pip install ${WHEEL}"
echo "  Remote:  gh release download ${TAG} -p '*.whl' -D /tmp && pip install /tmp/lsfm-*.whl"
