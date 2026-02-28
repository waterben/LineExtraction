#!/bin/bash
# =============================================================================
# setup_york_urban.sh - Download and prepare York Urban Line Segment Database
# =============================================================================
#
# This script downloads the York Urban Line Segment Database (102 images with
# ground truth line segments) and converts the MATLAB annotations to CSV format
# compatible with the GroundTruthLoader API.
#
# Source: https://www.elderlab.yorku.ca/resources/york-urban-line-segment-database/
# Reference: Denis, Elder & Estrada, ECCV 2008
#
# The dataset contains 102 urban images (640x480) with hand-labelled line
# segments satisfying the Manhattan assumption.
#
# Usage:
#   ./tools/scripts/setup_york_urban.sh [OPTIONS]
#
# Options:
#   --clean    Remove existing data before download
#   --help     Show this help message
#
# Output:
#   resources/datasets/YorkUrban/images/     - 102 JPEG images
#   resources/datasets/ground_truth/york_urban_gt.csv  - Ground truth CSV
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATASET_DIR="${WORKSPACE_ROOT}/resources/datasets/YorkUrban"
GT_DIR="${WORKSPACE_ROOT}/resources/datasets/ground_truth"
DOWNLOAD_URL="https://www.elderlab.yorku.ca/?sdm_process_download=1&download_id=8288"
CONVERTER="${SCRIPT_DIR}/convert_ground_truth.py"

# Minimum expected file size (1 MB) — the actual archive is ~30 MB;
# anything smaller is likely an HTML error page.
MIN_FILE_SIZE=1000000

CLEAN=false
MANUAL_FILE=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

show_help() {
    head -28 "$0" | tail -23
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --file)
            MANUAL_FILE="$2"
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

# Validate manual file if given
if [[ -n "$MANUAL_FILE" ]] && [[ ! -f "$MANUAL_FILE" ]]; then
    log_error "File not found: $MANUAL_FILE"
    exit 1
fi

# Clean if requested
if [[ "$CLEAN" == true ]]; then
    log_info "Cleaning existing York Urban data..."
    rm -rf "${DATASET_DIR}"
    rm -f "${GT_DIR}/york_urban_gt.csv"
fi

# Check if already set up
if [[ -d "${DATASET_DIR}/images" ]] && [[ -f "${GT_DIR}/york_urban_gt.csv" ]]; then
    IMAGE_COUNT=$(find "${DATASET_DIR}/images" -name "*.jpg" -o -name "*.png" 2>/dev/null | wc -l)
    if [[ "$IMAGE_COUNT" -ge 100 ]]; then
        log_info "York Urban dataset already set up (${IMAGE_COUNT} images found)"
        log_info "Use --clean to re-download"
        exit 0
    fi
fi

# Create directories
log_step "Creating directories..."
mkdir -p "${DATASET_DIR}/images"
mkdir -p "${GT_DIR}"

# Temporary directory for downloads
TMP_DIR=$(mktemp -d)
trap "rm -rf ${TMP_DIR}" EXIT

# Download the dataset
ZIP_FILE="${TMP_DIR}/YorkUrbanDB.zip"

if [[ -n "$MANUAL_FILE" ]]; then
    log_step "Using manually provided archive: ${MANUAL_FILE}"
    cp "$MANUAL_FILE" "${ZIP_FILE}"
else
    log_step "Downloading York Urban Line Segment Database..."
    log_info "URL: ${DOWNLOAD_URL}"

    DOWNLOAD_SUCCESS=false

    if command -v curl &>/dev/null; then
        # Try with SSL verification first, fall back to --insecure
        # (elderlab.yorku.ca often has expired/misconfigured certificates)
        if curl -L -o "${ZIP_FILE}" \
            --retry 3 \
            --retry-delay 5 \
            --connect-timeout 30 \
            --max-time 600 \
            --progress-bar \
            "${DOWNLOAD_URL}" 2>&1; then
            DOWNLOAD_SUCCESS=true
        else
            log_warn "SSL verification failed, retrying without certificate check..."
            if curl -L -o "${ZIP_FILE}" \
                --insecure \
                --retry 3 \
                --retry-delay 5 \
                --connect-timeout 30 \
                --max-time 600 \
                --progress-bar \
                "${DOWNLOAD_URL}" 2>&1; then
                DOWNLOAD_SUCCESS=true
            fi
        fi
    elif command -v wget &>/dev/null; then
        if wget -O "${ZIP_FILE}" \
            --tries=3 \
            --timeout=30 \
            --show-progress \
            "${DOWNLOAD_URL}" 2>&1; then
            DOWNLOAD_SUCCESS=true
        else
            log_warn "SSL verification failed, retrying without certificate check..."
            if wget -O "${ZIP_FILE}" \
                --no-check-certificate \
                --tries=3 \
                --timeout=30 \
                --show-progress \
                "${DOWNLOAD_URL}" 2>&1; then
                DOWNLOAD_SUCCESS=true
            fi
        fi
    else
        log_error "Neither curl nor wget found. Please install one of them."
        exit 1
    fi

    if [[ "$DOWNLOAD_SUCCESS" != true ]]; then
        log_error "Download failed. Please download manually and re-run with:"
        log_error "  $0 --file <path-to-zip>"
        log_error ""
        log_error "Download page:"
        log_error "  https://www.elderlab.yorku.ca/resources/york-urban-line-segment-database/"
        exit 1
    fi
fi

# Verify the download produced a real file
if [[ ! -s "${ZIP_FILE}" ]]; then
    log_error "Downloaded file is empty. Please download manually."
    exit 1
fi

FILE_SIZE=$(stat -c%s "${ZIP_FILE}" 2>/dev/null || stat -f%z "${ZIP_FILE}" 2>/dev/null || echo 0)
if [[ "$FILE_SIZE" -lt "$MIN_FILE_SIZE" ]]; then
    if head -c 200 "${ZIP_FILE}" | grep -qi '<html\|<!DOCTYPE'; then
        log_error "Downloaded file is an HTML page, not a ZIP archive."
    else
        log_error "Downloaded file is too small (${FILE_SIZE} bytes, expected >1 MB)."
    fi
    log_error "Please download manually and re-run with: $0 --file <path-to-zip>"
    log_error "Download page: https://www.elderlab.yorku.ca/resources/york-urban-line-segment-database/"
    exit 1
fi

log_info "Downloaded archive: $(numfmt --to=iec-i --suffix=B "${FILE_SIZE}" 2>/dev/null || echo "${FILE_SIZE} bytes")"

# Extract
log_step "Extracting dataset..."
EXTRACT_DIR="${TMP_DIR}/extracted"
mkdir -p "${EXTRACT_DIR}"
unzip -q "${ZIP_FILE}" -d "${EXTRACT_DIR}" 2>/dev/null || {
    log_error "Failed to extract ZIP file. It may be corrupted or in unexpected format."
    exit 1
}

# Find and copy images - the ZIP structure may vary
log_step "Organizing images..."
IMAGE_COUNT=0
# Look for images recursively in the extracted directory
while IFS= read -r img_file; do
    filename=$(basename "$img_file")
    cp "$img_file" "${DATASET_DIR}/images/${filename}"
    IMAGE_COUNT=$((IMAGE_COUNT + 1))
done < <(find "${EXTRACT_DIR}" -type f \( -iname "*.jpg" -o -iname "*.png" -o -iname "*.bmp" \) | sort)

log_info "Copied ${IMAGE_COUNT} images to ${DATASET_DIR}/images/"

if [[ "$IMAGE_COUNT" -eq 0 ]]; then
    log_error "No images found in the extracted archive."
    log_error "The archive structure may have changed."
    log_error "Contents of extracted directory:"
    find "${EXTRACT_DIR}" -maxdepth 3 -type f | head -20
    exit 1
fi

# Copy annotation files (.mat) for conversion
log_step "Organizing annotations..."
MAT_COUNT=0
mkdir -p "${DATASET_DIR}/annotations"
while IFS= read -r mat_file; do
    filename=$(basename "$mat_file")
    cp "$mat_file" "${DATASET_DIR}/annotations/${filename}"
    MAT_COUNT=$((MAT_COUNT + 1))
done < <(find "${EXTRACT_DIR}" -type f -iname "*.mat" | sort)

log_info "Copied ${MAT_COUNT} annotation files to ${DATASET_DIR}/annotations/"

# Convert annotations to CSV ground truth
log_step "Converting annotations to CSV ground truth..."

# Find Python3 in venv or system
PYTHON3=""
if [[ -x "${WORKSPACE_ROOT}/.venv/bin/python3" ]]; then
    PYTHON3="${WORKSPACE_ROOT}/.venv/bin/python3"
elif command -v python3 &>/dev/null; then
    PYTHON3="python3"
else
    log_error "Python3 not found. Install Python3 or set up the project venv."
    exit 1
fi

# Check that scipy is available (needed for .mat loading)
if ! "${PYTHON3}" -c "import scipy" 2>/dev/null; then
    log_warn "scipy not found in Python environment."
    log_warn "Installing scipy..."
    "${PYTHON3}" -m pip install scipy --quiet || {
        log_error "Failed to install scipy. Please install it manually:"
        log_error "  pip install scipy"
        exit 1
    }
fi

# Run the conversion script
"${PYTHON3}" "${CONVERTER}" \
    --dataset york_urban \
    --input-dir "${DATASET_DIR}/annotations" \
    --image-dir "${DATASET_DIR}/images" \
    --output "${GT_DIR}/york_urban_gt.csv" || {
    log_error "Ground truth conversion failed."
    log_error "You can run the converter manually later:"
    log_error "  python3 ${CONVERTER} --dataset york_urban \\"
    log_error "    --input-dir ${DATASET_DIR}/annotations \\"
    log_error "    --image-dir ${DATASET_DIR}/images \\"
    log_error "    --output ${GT_DIR}/york_urban_gt.csv"
    exit 1
}

# Summary
echo ""
log_info "======================================"
log_info "York Urban dataset setup complete!"
log_info "======================================"
log_info "Images:       ${DATASET_DIR}/images/ (${IMAGE_COUNT} files)"
log_info "Annotations:  ${DATASET_DIR}/annotations/ (${MAT_COUNT} files)"
log_info "Ground Truth: ${GT_DIR}/york_urban_gt.csv"
echo ""
log_info "Bazel usage:"
log_info "  data = [\"//resources/datasets:york_urban\"]"
echo ""
