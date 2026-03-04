#!/bin/bash
# =============================================================================
# setup_hpatches.sh - Download and prepare HPatches dataset
# =============================================================================
#
# This script downloads the HPatches dataset (Homography Patches Benchmark)
# for evaluating local feature descriptors and matchers.
#
# Source: https://github.com/hpatches/hpatches-dataset
# Reference: Balntas et al., "HPatches: A benchmark and evaluation of
#            handcrafted and learned local descriptors", CVPR 2017
#
# The dataset contains 116 sequences (57 illumination + 59 viewpoint changes),
# each with 6 images and 5 ground-truth homographies.
#
# Usage:
#   ./tools/scripts/setup_hpatches.sh [OPTIONS]
#
# Options:
#   --clean    Remove existing data before download
#   --help     Show this help message
#
# Output:
#   resources/datasets/HPatches/i_*/   - Illumination change sequences
#   resources/datasets/HPatches/v_*/   - Viewpoint change sequences
#
# Each sequence directory contains:
#   1.ppm - 6.ppm      Reference and target images
#   H_1_2 - H_1_6      Ground-truth homographies (3x3, space-separated)
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATASET_DIR="${WORKSPACE_ROOT}/resources/datasets/HPatches"
# Official mirror hosted on HuggingFace (the original icvl.ee.ic.ac.uk URL is defunct).
# See: https://github.com/hpatches/hpatches-dataset#full-image-sequences
DOWNLOAD_URL="https://huggingface.co/datasets/vbalnt/hpatches/resolve/main/hpatches-sequences-release.zip"

CLEAN=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# Functions
# =============================================================================

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Download and prepare the HPatches dataset."
    echo ""
    echo "Options:"
    echo "  --clean    Remove existing data before download"
    echo "  --help     Show this help message"
}

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# =============================================================================
# Parse arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN=true
            shift
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# =============================================================================
# Main
# =============================================================================

echo -e "${BLUE}=== HPatches Dataset Setup ===${NC}"
echo "Target directory: ${DATASET_DIR}"
echo ""

# Clean if requested
if [[ "${CLEAN}" == "true" ]]; then
    log_warn "Removing existing HPatches data..."
    rm -rf "${DATASET_DIR}"
fi

# Check if already downloaded
if [[ -d "${DATASET_DIR}" ]]; then
    n_seq=$(find "${DATASET_DIR}" -maxdepth 1 -type d \( -name 'i_*' -o -name 'v_*' \) | wc -l)
    if [[ "${n_seq}" -ge 100 ]]; then
        log_info "HPatches already present (${n_seq} sequences). Use --clean to re-download."
        exit 0
    fi
    log_warn "Partial download found (${n_seq} sequences). Re-downloading..."
    rm -rf "${DATASET_DIR}"
fi

mkdir -p "${DATASET_DIR}"

# Download
ARCHIVE="${DATASET_DIR}/hpatches-sequences-release.zip"
log_info "Downloading HPatches (~1.3 GB)..."
if command -v wget &>/dev/null; then
    wget -q --show-progress -O "${ARCHIVE}" "${DOWNLOAD_URL}"
elif command -v curl &>/dev/null; then
    curl -L --progress-bar -o "${ARCHIVE}" "${DOWNLOAD_URL}"
else
    log_error "Neither wget nor curl found. Cannot download."
    exit 1
fi

# Verify download
if [[ ! -f "${ARCHIVE}" ]]; then
    log_error "Download failed — archive not found."
    exit 1
fi

FILESIZE=$(stat -c%s "${ARCHIVE}" 2>/dev/null || stat -f%z "${ARCHIVE}" 2>/dev/null)
if [[ "${FILESIZE}" -lt 100000000 ]]; then
    log_error "Downloaded file is suspiciously small (${FILESIZE} bytes). Possible error."
    rm -f "${ARCHIVE}"
    exit 1
fi

# Extract (zip archive — uses a top-level hpatches-sequences-release/ directory)
log_info "Extracting archive..."
TMPDIR="${DATASET_DIR}/_extract_tmp"
mkdir -p "${TMPDIR}"
unzip -q "${ARCHIVE}" -d "${TMPDIR}"

# Move contents from the top-level directory inside the zip into DATASET_DIR.
# The archive unpacks to hpatches-sequences-release/{i_*,v_*,...}.
INNER="${TMPDIR}/hpatches-sequences-release"
if [[ -d "${INNER}" ]]; then
    mv "${INNER}"/* "${DATASET_DIR}/"
else
    # Fallback: sequences directly in the tmp folder
    mv "${TMPDIR}"/* "${DATASET_DIR}/" 2>/dev/null || true
fi
rm -rf "${TMPDIR}"

# Clean up archive
rm -f "${ARCHIVE}"

# Verify
n_seq=$(find "${DATASET_DIR}" -maxdepth 1 -type d \( -name 'i_*' -o -name 'v_*' \) | wc -l)
log_info "Extraction complete: ${n_seq} sequences."

if [[ "${n_seq}" -lt 100 ]]; then
    log_warn "Expected ~116 sequences, found ${n_seq}. Dataset might be incomplete."
fi

echo ""
echo -e "${GREEN}=== HPatches setup complete ===${NC}"
echo "Dataset location: ${DATASET_DIR}"
echo "Sequences: ${n_seq} (57 illumination + 59 viewpoint)"
echo ""
echo "Usage in Python:"
echo "  from lsfm.data import TestImages"
echo "  images = TestImages()"
echo "  ref, target, H = images.hpatches_pair('v_bark', 2)"
