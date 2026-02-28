#!/bin/bash
# =============================================================================
# setup_wireframe.sh - Download and prepare Wireframe Dataset
# =============================================================================
#
# This script downloads the Wireframe dataset (Huang et al., CVPR 2018) and
# converts the annotations to CSV format compatible with GroundTruthLoader.
#
# Source: https://github.com/huangkuns/wireframe
# Reference: Huang et al., "Learning to Parse Wireframes in Images of
#            Man-Made Environments", CVPR 2018
#
# The dataset contains 5462 images with vectorized wireframe annotations
# (junctions + edges). This script downloads the test split (462 images)
# by default for evaluation purposes.
#
# The dataset is hosted on OneDrive / BaiduPan. If automatic download fails,
# download manually and pass via --file <path-to-zip>.
#
# Usage:
#   ./tools/scripts/setup_wireframe.sh [OPTIONS]
#
# Options:
#   --split test     Download test split only (462 images, default)
#   --split train    Download training split only (5000 images)
#   --split all      Download both splits
#   --file <path>    Use a manually downloaded archive instead of downloading
#   --clean          Remove existing data before download
#   --help           Show this help message
#
# Output:
#   resources/datasets/Wireframe/images/     - JPEG images
#   resources/datasets/ground_truth/wireframe_gt.csv  - Ground truth CSV
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATASET_DIR="${WORKSPACE_ROOT}/resources/datasets/Wireframe"
GT_DIR="${WORKSPACE_ROOT}/resources/datasets/ground_truth"
CONVERTER="${SCRIPT_DIR}/convert_ground_truth.py"

# Primary: HuggingFace mirror from LCNN project (zhou13/lcnn)
# This is a reliable, publicly accessible mirror of the raw wireframe dataset.
HF_URL="https://huggingface.co/yichaozhou/lcnn/resolve/main/Data/wireframe_raw.tar.xz"

# Fallback: Original OneDrive sharing URL (from huangkuns/wireframe repo)
# Note: This link has been unreliable (403/expired) since the account migrated.
ONEDRIVE_SHARE_URL="https://1drv.ms/u/s!AqQBtmo8Qg_9uHpjzIybaIfyJ-Zf?e=Fofbch"

# Minimum expected file size for the archive (10 MB) — the actual file is
# several hundred MB; anything smaller is most likely an HTML error page.
MIN_FILE_SIZE=10000000

SPLIT="test"
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
    head -38 "$0" | tail -33
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --split)
            SPLIT="$2"
            shift 2
            ;;
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

# Validate split
case $SPLIT in
    test|train|all)
        ;;
    *)
        log_error "Invalid split: $SPLIT (must be test, train, or all)"
        exit 1
        ;;
esac

# Validate manual file if given
if [[ -n "$MANUAL_FILE" ]] && [[ ! -f "$MANUAL_FILE" ]]; then
    log_error "File not found: $MANUAL_FILE"
    exit 1
fi

# Clean if requested
if [[ "$CLEAN" == true ]]; then
    log_info "Cleaning existing Wireframe data..."
    rm -rf "${DATASET_DIR}"
    rm -f "${GT_DIR}/wireframe_gt.csv"
fi

# Check if already set up
if [[ -d "${DATASET_DIR}/images" ]] && [[ -f "${GT_DIR}/wireframe_gt.csv" ]]; then
    IMAGE_COUNT=$(find "${DATASET_DIR}/images" -name "*.jpg" -o -name "*.png" 2>/dev/null | wc -l)
    if [[ "$IMAGE_COUNT" -ge 100 ]]; then
        log_info "Wireframe dataset already set up (${IMAGE_COUNT} images found)"
        log_info "Use --clean to re-download"
        exit 0
    fi
fi

# Create directories
log_step "Creating directories..."
mkdir -p "${DATASET_DIR}/images"
mkdir -p "${DATASET_DIR}/annotations"
mkdir -p "${GT_DIR}"

# Temporary directory for downloads
TMP_DIR=$(mktemp -d)
trap "rm -rf ${TMP_DIR}" EXIT

ARCHIVE_FILE="${TMP_DIR}/wireframe_raw.tar.xz"

if [[ -n "$MANUAL_FILE" ]]; then
    # ---- Use manually provided archive -----------------------------------
    log_step "Using manually provided archive: ${MANUAL_FILE}"
    cp "$MANUAL_FILE" "${ARCHIVE_FILE}"
else
    # ---- Automatic download -----------------------------------------------
    log_step "Downloading Wireframe dataset (split: ${SPLIT})..."
    log_info "Source: HuggingFace (LCNN project mirror)"
    log_warn "If automatic download fails, download manually and re-run with:"
    log_warn "  $0 --file <path-to-archive>"
    log_warn ""
    log_warn "Alternative download sources:"
    log_warn "  HuggingFace: ${HF_URL}"
    log_warn "  OneDrive:    ${ONEDRIVE_SHARE_URL}"
    log_warn "  BaiduPan:    https://pan.baidu.com/s/11CKr5s0zHnuVKsJVXianxA?pwd=wf18"
    echo ""

    DOWNLOAD_SUCCESS=false

    # Try HuggingFace first (most reliable)
    if command -v curl &>/dev/null; then
        log_info "Downloading from HuggingFace (~1.6 GB)..."
        if curl -L -o "${ARCHIVE_FILE}" \
            --retry 3 \
            --retry-delay 5 \
            --connect-timeout 30 \
            --max-time 3600 \
            --progress-bar \
            "${HF_URL}" 2>&1; then
            DOWNLOAD_SUCCESS=true
        fi
    elif command -v wget &>/dev/null; then
        log_info "Downloading from HuggingFace (~1.6 GB)..."
        if wget -O "${ARCHIVE_FILE}" \
            --tries=3 \
            --timeout=30 \
            --show-progress \
            "${HF_URL}" 2>&1; then
            DOWNLOAD_SUCCESS=true
        fi
    else
        log_error "Neither curl nor wget found. Please install one of them."
        exit 1
    fi

    if [[ "$DOWNLOAD_SUCCESS" != true ]]; then
        log_error "Automatic download failed."
        log_error "Please download the dataset manually and re-run with:"
        log_error "  $0 --file <path-to-archive>"
        exit 1
    fi
fi

# Verify the download is a real archive, not an HTML error page
if [[ ! -s "${ARCHIVE_FILE}" ]]; then
    log_error "Downloaded file is empty. Please download manually."
    exit 1
fi

FILE_SIZE=$(stat -c%s "${ARCHIVE_FILE}" 2>/dev/null || stat -f%z "${ARCHIVE_FILE}" 2>/dev/null || echo 0)
if [[ "$FILE_SIZE" -lt "$MIN_FILE_SIZE" ]]; then
    # Check if it's actually an HTML page (common with failed OneDrive links)
    if head -c 200 "${ARCHIVE_FILE}" | grep -qi '<html\|<!DOCTYPE'; then
        log_error "Downloaded file is an HTML page, not an archive."
        log_error "The download URL may have expired or changed."
    else
        log_error "Downloaded file is too small (${FILE_SIZE} bytes, expected >10 MB)."
        log_error "The download may be incomplete or corrupted."
    fi
    log_error ""
    log_error "Please download the dataset manually and re-run with:"
    log_error "  $0 --file <path-to-archive>"
    log_error ""
    log_error "Download sources:"
    log_error "  HuggingFace: ${HF_URL}"
    log_error "  OneDrive:    ${ONEDRIVE_SHARE_URL}"
    log_error "  BaiduPan:    https://pan.baidu.com/s/11CKr5s0zHnuVKsJVXianxA?pwd=wf18"
    exit 1
fi

log_info "Downloaded archive: $(numfmt --to=iec-i --suffix=B "${FILE_SIZE}" 2>/dev/null || echo "${FILE_SIZE} bytes")"

# Extract — detect format automatically (.tar.xz, .tar.gz, .zip)
log_step "Extracting dataset..."
EXTRACT_DIR="${TMP_DIR}/extracted"
mkdir -p "${EXTRACT_DIR}"

if file "${ARCHIVE_FILE}" | grep -qi 'xz'; then
    tar -xJf "${ARCHIVE_FILE}" -C "${EXTRACT_DIR}" || {
        log_error "Failed to extract .tar.xz archive."
        exit 1
    }
elif file "${ARCHIVE_FILE}" | grep -qi 'gzip'; then
    tar -xzf "${ARCHIVE_FILE}" -C "${EXTRACT_DIR}" || {
        log_error "Failed to extract .tar.gz archive."
        exit 1
    }
elif file "${ARCHIVE_FILE}" | grep -qi 'zip'; then
    unzip -q "${ARCHIVE_FILE}" -d "${EXTRACT_DIR}" || {
        log_error "Failed to extract .zip archive."
        exit 1
    }
else
    # Try common formats in order
    if ! tar -xJf "${ARCHIVE_FILE}" -C "${EXTRACT_DIR}" 2>/dev/null; then
        if ! tar -xf "${ARCHIVE_FILE}" -C "${EXTRACT_DIR}" 2>/dev/null; then
            if ! unzip -q "${ARCHIVE_FILE}" -d "${EXTRACT_DIR}" 2>/dev/null; then
                log_error "Failed to extract archive. Unrecognized format."
                exit 1
            fi
        fi
    fi
fi

# Locate the images directory and annotation JSONs inside the extracted tree.
# The HuggingFace archive extracts to wireframe_raw/{images/,train.json,valid.json}.
# Older v1.1.zip archives may have train/ and test/ subdirectories instead.
SRC_IMAGE_DIR=$(find "${EXTRACT_DIR}" -type d -iname "images" | head -1)

# Determine which JSON annotation files map to the requested split.
# LCNN format: valid.json = test split (462), train.json = train split (5000).
# Map our --split flag to file names:
SPLIT_JSONS=()
case $SPLIT in
    test)  SPLIT_JSONS=("valid.json") ;;
    train) SPLIT_JSONS=("train.json") ;;
    all)   SPLIT_JSONS=("valid.json" "train.json") ;;
esac

# Copy split-filtered annotations first — we need them to know which images
# belong to the requested split.
log_step "Organizing annotations (split: ${SPLIT})..."
ANNO_COUNT=0
FOUND_BATCH_JSON=false
for json_name in "${SPLIT_JSONS[@]}"; do
    SRC_JSON=$(find "${EXTRACT_DIR}" -type f -name "$json_name" | head -1)
    if [[ -n "$SRC_JSON" ]]; then
        cp "$SRC_JSON" "${DATASET_DIR}/annotations/${json_name}"
        ANNO_COUNT=$((ANNO_COUNT + 1))
        FOUND_BATCH_JSON=true
    fi
done

# Also grab any .pkl files (original wireframe format)
while IFS= read -r anno_file; do
    filename=$(basename "$anno_file")
    cp "$anno_file" "${DATASET_DIR}/annotations/${filename}"
    ANNO_COUNT=$((ANNO_COUNT + 1))
done < <(find "${EXTRACT_DIR}" -type f -iname "*.pkl" | sort)

log_info "Copied ${ANNO_COUNT} annotation files to ${DATASET_DIR}/annotations/"

# --- Organize images ---
log_step "Organizing images (split: ${SPLIT})..."
IMAGE_COUNT=0

if [[ "$FOUND_BATCH_JSON" == true ]] && [[ -n "$SRC_IMAGE_DIR" ]]; then
    # LCNN format: use the JSON filenames to select only matching images.
    # Build a list of needed image filenames from the annotation JSONs.
    NEEDED_IMAGES="${TMP_DIR}/needed_images.txt"
    for json_name in "${SPLIT_JSONS[@]}"; do
        json_path="${DATASET_DIR}/annotations/${json_name}"
        if [[ -f "$json_path" ]]; then
            python3 -c "
import json, sys
with open('${json_path}') as f:
    data = json.load(f)
for entry in data:
    fn = entry.get('filename', entry.get('imagename', ''))
    if fn:
        print(fn)
" >> "${NEEDED_IMAGES}"
        fi
    done

    if [[ -s "${NEEDED_IMAGES}" ]]; then
        while IFS= read -r img_name; do
            src_file="${SRC_IMAGE_DIR}/${img_name}"
            if [[ -f "$src_file" ]]; then
                cp "$src_file" "${DATASET_DIR}/images/${img_name}"
                IMAGE_COUNT=$((IMAGE_COUNT + 1))
            fi
        done < "${NEEDED_IMAGES}"
    fi
else
    # Fallback: look for train/test subdirectories (v1.1.zip format)
    copy_images_from_dir() {
        local src_dir=$1
        if [[ -d "$src_dir" ]]; then
            while IFS= read -r img_file; do
                filename=$(basename "$img_file")
                cp "$img_file" "${DATASET_DIR}/images/${filename}"
                IMAGE_COUNT=$((IMAGE_COUNT + 1))
            done < <(find "$src_dir" -maxdepth 1 -type f \( -iname "*.jpg" -o -iname "*.png" \) | sort)
        fi
    }

    if [[ "$SPLIT" == "test" ]] || [[ "$SPLIT" == "all" ]]; then
        while IFS= read -r d; do
            copy_images_from_dir "$d"
        done < <(find "${EXTRACT_DIR}" -type d -iname "test")
    fi

    if [[ "$SPLIT" == "train" ]] || [[ "$SPLIT" == "all" ]]; then
        while IFS= read -r d; do
            copy_images_from_dir "$d"
        done < <(find "${EXTRACT_DIR}" -type d -iname "train")
    fi

    # If no split dirs found, copy all images recursively
    if [[ "$IMAGE_COUNT" -eq 0 ]]; then
        log_warn "No split directories found, copying all images..."
        while IFS= read -r img_file; do
            filename=$(basename "$img_file")
            cp "$img_file" "${DATASET_DIR}/images/${filename}"
            IMAGE_COUNT=$((IMAGE_COUNT + 1))
        done < <(find "${EXTRACT_DIR}" -type f \( -iname "*.jpg" -o -iname "*.png" \) | sort)
    fi
fi

log_info "Copied ${IMAGE_COUNT} images to ${DATASET_DIR}/images/"

if [[ "$IMAGE_COUNT" -eq 0 ]]; then
    log_error "No images found in the extracted archive."
    log_error "The archive structure may have changed. Contents:"
    find "${EXTRACT_DIR}" -maxdepth 3 -type f | head -20
    exit 1
fi

# Convert annotations to CSV ground truth
log_step "Converting annotations to CSV ground truth..."

# Find Python3
PYTHON3=""
if [[ -x "${WORKSPACE_ROOT}/.venv/bin/python3" ]]; then
    PYTHON3="${WORKSPACE_ROOT}/.venv/bin/python3"
elif command -v python3 &>/dev/null; then
    PYTHON3="python3"
else
    log_error "Python3 not found."
    exit 1
fi

# Run the conversion script
"${PYTHON3}" "${CONVERTER}" \
    --dataset wireframe \
    --input-dir "${DATASET_DIR}/annotations" \
    --image-dir "${DATASET_DIR}/images" \
    --output "${GT_DIR}/wireframe_gt.csv" || {
    log_warn "Automatic ground truth conversion failed."
    log_warn "You can run the converter manually later:"
    log_warn "  python3 ${CONVERTER} --dataset wireframe \\"
    log_warn "    --input-dir ${DATASET_DIR}/annotations \\"
    log_warn "    --image-dir ${DATASET_DIR}/images \\"
    log_warn "    --output ${GT_DIR}/wireframe_gt.csv"
}

# Summary
echo ""
log_info "======================================"
log_info "Wireframe dataset setup complete!"
log_info "======================================"
log_info "Images:       ${DATASET_DIR}/images/ (${IMAGE_COUNT} files)"
log_info "Annotations:  ${DATASET_DIR}/annotations/ (${ANNO_COUNT} files)"
if [[ -f "${GT_DIR}/wireframe_gt.csv" ]]; then
    GT_LINES=$(wc -l < "${GT_DIR}/wireframe_gt.csv")
    log_info "Ground Truth: ${GT_DIR}/wireframe_gt.csv (${GT_LINES} lines)"
fi
echo ""
log_info "Bazel usage:"
log_info "  data = [\"//resources/datasets:wireframe\"]"
echo ""
