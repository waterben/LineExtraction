#!/bin/bash
# =============================================================================
# setup_mdb_dataset.sh - Download and prepare Middlebury Stereo Dataset
# =============================================================================
#
# This script downloads the Middlebury Stereo Evaluation 2014 dataset and
# prepares it for use with the LineExtraction performance tests.
#
# The dataset is NOT included as a Bazel dependency due to its large size
# (~2GB for all resolutions). Instead, this script downloads and extracts
# only the necessary images (im0.png from "perfect" exposure).
#
# Source: https://vision.middlebury.edu/stereo/data/scenes2014/
#
# Usage:
#   ./tools/scripts/setup_mdb_dataset.sh [--resolution Q|H|F|all]
#
# Options:
#   --resolution Q     Quarter resolution only (~50MB)
#   --resolution H     Half resolution only (~200MB)
#   --resolution F     Full resolution only (~800MB)
#   --resolution all   All resolutions (default, ~1GB)
#   --clean            Remove existing MDB data before download
#   --help             Show this help message
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MDB_DIR="${WORKSPACE_ROOT}/resources/datasets/MDB"
MDB_BASE_URL="https://vision.middlebury.edu/stereo/data/scenes2014/zip"

# Scene list with version info
# Format: "scene:version" - most scenes use "perfect", some only have older versions
declare -A SCENES=(
    ["Adirondack"]="perfect"
    ["ArtL"]="perfect"
    ["Jadeplant"]="perfect"
    ["Motorcycle"]="perfect"
    ["MotorcycleE"]="perfect"
    ["Piano"]="perfect"
    ["PianoL"]="perfect"
    ["Pipes"]="perfect"
    ["Playroom"]="perfect"
    ["Playtable"]="perfect"
    ["PlaytableP"]="perfect"
    ["Recycle"]="perfect"
    ["Shelves"]="perfect"
    ["Teddy"]="perfect"
    ["Vintage"]="perfect"
    # Training scenes
    ["Backpack"]="perfect"
    ["Bicycle1"]="perfect"
    ["Cable"]="perfect"
    ["Classroom1"]="perfect"
    ["Couch"]="perfect"
    ["Flowers"]="perfect"
    ["Mask"]="perfect"
    ["Shopvac"]="perfect"
    ["Sticks"]="perfect"
    ["Storage"]="perfect"
    ["Sword1"]="perfect"
    ["Sword2"]="perfect"
    ["Umbrella"]="perfect"
    # Additional scenes (some may have different versions)
    ["Australia"]="perfect"
    ["Crusade"]="perfect"
    ["CrusadeP"]="perfect"
    ["Djembe"]="perfect"
    ["DjembeL"]="perfect"
    ["Hoops"]="perfect"
    ["Livingroom"]="perfect"
    ["Newkuba"]="perfect"
    ["Plants"]="perfect"
    ["Staircase"]="perfect"
    # Scenes with only v1/v2 available (no "perfect")
    ["Bicycle2"]="imperfect"
    ["Classroom2"]="imperfect"
)

# Resolutions to download
RESOLUTIONS="all"
CLEAN=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    head -30 "$0" | tail -25
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --resolution)
            RESOLUTIONS="$2"
            shift 2
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --help|-h)
            show_help
            ;;
        *)
            log_error "Unknown option: $1"
            show_help
            ;;
    esac
done

# Validate resolution
case $RESOLUTIONS in
    Q|H|F|all)
        ;;
    *)
        log_error "Invalid resolution: $RESOLUTIONS (must be Q, H, F, or all)"
        exit 1
        ;;
esac

# Clean if requested
if [[ "$CLEAN" == true ]]; then
    log_info "Cleaning existing MDB data..."
    rm -rf "${MDB_DIR}"
fi

# Create directories
log_info "Creating MDB directories..."
mkdir -p "${MDB_DIR}/MiddEval3-Q"
mkdir -p "${MDB_DIR}/MiddEval3-H"
mkdir -p "${MDB_DIR}/MiddEval3-F"

# Temporary directory for downloads
TMP_DIR=$(mktemp -d)
trap "rm -rf ${TMP_DIR}" EXIT

download_scene() {
    local scene=$1
    local res=$2
    local version=${SCENES[$scene]}
    local output_dir="${MDB_DIR}/MiddEval3-${res}"
    local output_file="${output_dir}/${scene}.png"

    # Skip if already exists
    if [[ -f "$output_file" ]]; then
        log_info "  [SKIP] ${scene} (${res}) - already exists"
        return 0
    fi

    # Construct URL based on version
    local url
    if [[ "$version" == "perfect" ]]; then
        url="${MDB_BASE_URL}/${scene}-${version}.zip"
    else
        # For imperfect scenes, try to get the best available
        url="${MDB_BASE_URL}/${scene}-imperfect.zip"
    fi

    local zip_file="${TMP_DIR}/${scene}-${version}.zip"

    log_info "  Downloading ${scene} (${res})..."

    # Download with retry
    local retries=3
    local success=false
    for ((i=1; i<=retries; i++)); do
        if curl -fsSL -o "$zip_file" "$url" 2>/dev/null; then
            success=true
            break
        fi
        log_warn "    Retry $i/$retries for ${scene}..."
        sleep 1
    done

    if [[ "$success" != true ]]; then
        log_warn "  [FAIL] Could not download ${scene}"
        return 1
    fi

    # Extract only the im0.png for the requested resolution
    local extract_path
    case $res in
        Q) extract_path="${scene}-${version}/im0.png" ;;
        H) extract_path="${scene}-${version}/im0.png" ;;
        F) extract_path="${scene}-${version}/im0.png" ;;
    esac

    # Check what's in the archive and extract appropriately
    if unzip -l "$zip_file" | grep -q "im0.png"; then
        # Extract to temp, then copy the right resolution
        local extract_dir="${TMP_DIR}/extract_${scene}"
        mkdir -p "$extract_dir"
        unzip -q -o "$zip_file" -d "$extract_dir"

        # Find im0.png (structure varies)
        local im0_path
        im0_path=$(find "$extract_dir" -name "im0.png" -type f | head -1)

        if [[ -n "$im0_path" ]]; then
            cp "$im0_path" "$output_file"
            log_info "  [OK] ${scene} (${res})"
        else
            log_warn "  [FAIL] im0.png not found in ${scene}"
            return 1
        fi

        rm -rf "$extract_dir"
    else
        log_warn "  [FAIL] Invalid archive for ${scene}"
        return 1
    fi

    # Cleanup zip
    rm -f "$zip_file"
    return 0
}

# Determine which resolutions to download
res_list=()
case $RESOLUTIONS in
    Q)   res_list=("Q") ;;
    H)   res_list=("H") ;;
    F)   res_list=("F") ;;
    all) res_list=("Q" "H" "F") ;;
esac

# Download each scene for each resolution
log_info "Downloading Middlebury Stereo Dataset..."
log_info "Resolutions: ${res_list[*]}"
log_info "Target: ${MDB_DIR}"
echo ""

total_scenes=${#SCENES[@]}
downloaded=0
failed=0

for res in "${res_list[@]}"; do
    log_info "Processing MiddEval3-${res}..."

    for scene in "${!SCENES[@]}"; do
        if download_scene "$scene" "$res"; then
            ((downloaded++)) || true
        else
            ((failed++)) || true
        fi
    done
    echo ""
done

# Summary
echo ""
log_info "========================================"
log_info "MDB Dataset Setup Complete"
log_info "========================================"
log_info "Downloaded: $downloaded images"
if [[ $failed -gt 0 ]]; then
    log_warn "Failed: $failed images"
fi
log_info "Location: ${MDB_DIR}"
echo ""
log_info "To use in Bazel builds, add to your target:"
echo '    data = ["//resources/datasets:mdb_q"]  # or mdb_h, mdb_f'
