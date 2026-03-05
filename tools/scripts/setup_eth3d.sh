#!/bin/bash
# =============================================================================
# setup_eth3d.sh - Download ETH3D multi-view stereo benchmark data
# =============================================================================
#
# Downloads selected scenes from the ETH3D multi-view stereo benchmark.
# These provide calibrated multi-view image sets with ground truth point
# clouds for evaluating 3D line reconstruction.
#
# The archives are .7z format hosted at https://www.eth3d.net/data/.
# Extraction uses p7zip (7z) if available, otherwise falls back to the
# py7zr Python package (auto-installed into the project venv).
#
# Source: https://www.eth3d.net/
# Reference: Schops et al., "A Multi-View Stereo Benchmark with
#   High-Resolution Images and Multi-Camera Videos", CVPR 2017.
#
# Usage:
#   ./tools/scripts/setup_eth3d.sh [OPTIONS]
#
# Options:
#   --scenes LIST   Comma-separated scene names
#                   (default: courtyard,delivery_area,electro)
#   --type TYPE     "training" or "test" (default: training)
#   --clean         Remove existing data before download
#   --help          Show this help message
#
# Output:
#   resources/datasets/ETH3D/<scene>/images/       - Multi-view images
#   resources/datasets/ETH3D/<scene>/cameras.txt   - Camera parameters
#   resources/datasets/ETH3D/<scene>/images.txt    - Image list
#   resources/datasets/ETH3D/<scene>/cameras.json  - Converted camera params
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATASET_DIR="${WORKSPACE_ROOT}/resources/datasets/ETH3D"
VENV_PYTHON="${WORKSPACE_ROOT}/.venv/bin/python"

# Defaults
SCENES="courtyard,delivery_area,electro"
SPLIT="training"
CLEAN=false

# ETH3D download base URL
BASE_URL="https://www.eth3d.net/data"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "${BLUE}[STEP]${NC} $1"; }

show_help() {
    head -35 "$0" | tail -30
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --scenes)
            SCENES="$2"
            shift 2
            ;;
        --type)
            SPLIT="$2"
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

# --------------------------------------------------------------------------
# Pre-flight checks
# --------------------------------------------------------------------------

if ! command -v curl &> /dev/null && ! command -v wget &> /dev/null; then
    log_error "Neither curl nor wget found. Install one and retry."
    exit 1
fi

# Determine extraction method for .7z archives.
USE_7Z=false
USE_PY7ZR=false

if command -v 7z &> /dev/null; then
    USE_7Z=true
elif command -v 7za &> /dev/null; then
    USE_7Z=true          # 7za is CLI-compatible with 7z
elif [[ -x "$VENV_PYTHON" ]]; then
    if "$VENV_PYTHON" -c "import py7zr" 2>/dev/null; then
        USE_PY7ZR=true
    else
        log_warn "py7zr not found in venv — installing..."
        if command -v uv &> /dev/null; then
            uv pip install --quiet py7zr
        else
            "$VENV_PYTHON" -m pip install --quiet py7zr
        fi
        if "$VENV_PYTHON" -c "import py7zr" 2>/dev/null; then
            USE_PY7ZR=true
        else
            log_error "Failed to install py7zr. Install p7zip-full or py7zr manually."
            exit 1
        fi
    fi
else
    log_error "No .7z extraction tool found."
    log_error "Install p7zip-full (apt install p7zip-full) or set up the Python venv."
    exit 1
fi

download_file() {
    local url="$1"
    local dest="$2"
    if command -v curl &> /dev/null; then
        curl -fSL -o "$dest" "$url"
    else
        wget -O "$dest" "$url"
    fi
}

extract_7z() {
    local archive="$1"
    local dest_dir="$2"
    if [[ "$USE_7Z" == true ]]; then
        if command -v 7z &> /dev/null; then
            7z x -o"$dest_dir" -y "$archive" > /dev/null
        else
            7za x -o"$dest_dir" -y "$archive" > /dev/null
        fi
    elif [[ "$USE_PY7ZR" == true ]]; then
        "$VENV_PYTHON" -c "
import py7zr, sys
with py7zr.SevenZipFile(sys.argv[1], 'r') as z:
    z.extractall(sys.argv[2])
" "$archive" "$dest_dir"
    fi
}

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

if [[ "$CLEAN" == true ]] && [[ -d "$DATASET_DIR" ]]; then
    log_warn "Cleaning existing ETH3D data..."
    rm -rf "$DATASET_DIR"
fi

mkdir -p "$DATASET_DIR"

IFS=',' read -ra SCENE_ARRAY <<< "$SCENES"

for scene in "${SCENE_ARRAY[@]}"; do
    scene_dir="${DATASET_DIR}/${scene}"
    marker="${scene_dir}/.download_complete"

    if [[ -f "$marker" ]]; then
        log_info "Scene ${scene} already downloaded, skipping."
        continue
    fi

    log_step "Downloading scene: ${scene}"
    mkdir -p "$scene_dir"

    # ETH3D per-scene undistorted images archive:
    #   https://www.eth3d.net/data/{scene}_dslr_undistorted.7z
    archive_name="${scene}_dslr_undistorted.7z"
    archive_url="${BASE_URL}/${archive_name}"
    archive_path="${DATASET_DIR}/${archive_name}"

    log_info "  Fetching ${archive_url}..."
    if download_file "$archive_url" "$archive_path"; then
        log_info "  Extracting (${archive_name})..."
        extract_7z "$archive_path" "$scene_dir"
        rm -f "$archive_path"

        # ETH3D archives extract into a subfolder named after the scene;
        # flatten so that images/ and calibration/ live directly under scene_dir.
        inner_dir="${scene_dir}/${scene}"
        if [[ -d "$inner_dir" ]]; then
            # Move contents up one level.
            shopt -s dotglob
            mv "$inner_dir"/* "$scene_dir/" 2>/dev/null || true
            shopt -u dotglob
            rmdir "$inner_dir" 2>/dev/null || true
        fi

        touch "$marker"
        log_info "  Scene ${scene} extracted."
    else
        log_warn "  Download failed for ${scene}."
        log_warn "  ETH3D may require manual download (registration)."
        log_warn "  See: https://www.eth3d.net/datasets"
        log_warn "  Place extracted data in: ${scene_dir}/"
        continue
    fi

    # Try to parse COLMAP cameras.txt → cameras.json for convenience.
    # ETH3D undistorted archives contain:
    #   dslr_calibration_undistorted/cameras.txt  (COLMAP cameras)
    #   dslr_calibration_undistorted/images.txt   (COLMAP images)
    #   images/dslr_images_undistorted/*.JPG       (actual images)
    colmap_cameras="${scene_dir}/dslr_calibration_undistorted/cameras.txt"
    colmap_images="${scene_dir}/dslr_calibration_undistorted/images.txt"

    if [[ -f "$colmap_cameras" ]] && [[ -f "$colmap_images" ]]; then
        log_info "  Converting COLMAP calibration to cameras.json..."
        if [[ -x "$VENV_PYTHON" ]]; then
            "$VENV_PYTHON" -c "
import json, sys, os
from pathlib import Path

import numpy as np

scene_dir = Path('${scene_dir}')
cam_file = scene_dir / 'dslr_calibration_undistorted' / 'cameras.txt'
img_file = scene_dir / 'dslr_calibration_undistorted' / 'images.txt'

# Parse cameras.txt (COLMAP format)
cameras = {}
with open(cam_file) as f:
    for line in f:
        line = line.strip()
        if line.startswith('#') or not line:
            continue
        parts = line.split()
        cam_id = int(parts[0])
        model = parts[1]
        w, h = int(parts[2]), int(parts[3])
        params = [float(x) for x in parts[4:]]
        if model == 'PINHOLE':
            fx, fy, cx, cy = params[:4]
        elif model in ('SIMPLE_PINHOLE', 'SIMPLE_RADIAL'):
            fx = fy = params[0]
            cx, cy = params[1], params[2]
        else:
            fx = fy = params[0] if params else 500.0
            cx, cy = w / 2, h / 2
        cameras[cam_id] = {
            'K': [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
            'width': w,
            'height': h,
        }

# Parse images.txt (COLMAP format — every other line is an image)
frames = []
with open(img_file) as f:
    lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]

for i in range(0, len(lines), 2):
    parts = lines[i].split()
    if len(parts) < 10:
        continue
    qw, qx, qy, qz = [float(x) for x in parts[1:5]]
    tx, ty, tz = [float(x) for x in parts[5:8]]
    cam_id = int(parts[8])
    img_name = parts[9]

    # Quaternion to rotation matrix
    q = np.array([qw, qx, qy, qz])
    q = q / np.linalg.norm(q)
    w_, x_, y_, z_ = q
    R = np.array([
        [1 - 2*(y_**2 + z_**2), 2*(x_*y_ - w_*z_), 2*(x_*z_ + w_*y_)],
        [2*(x_*y_ + w_*z_), 1 - 2*(x_**2 + z_**2), 2*(y_*z_ - w_*x_)],
        [2*(x_*z_ - w_*y_), 2*(y_*z_ + w_*x_), 1 - 2*(x_**2 + y_**2)],
    ])

    cam = cameras.get(cam_id, {})
    frames.append({
        'image': img_name,
        'R': R.tolist(),
        't': [tx, ty, tz],
        **cam,
    })

out_path = scene_dir / 'cameras.json'
with open(out_path, 'w') as f:
    json.dump({'scene': '${scene}', 'n_images': len(frames), 'frames': frames}, f, indent=2)
print(f'  Wrote cameras.json with {len(frames)} frames')
" 2>&1 || log_warn "  COLMAP conversion failed."
        else
            log_warn "  Python venv not found; COLMAP conversion skipped."
        fi
    else
        log_warn "  COLMAP calibration files not found in extracted data."
    fi

    log_info "  Scene ${scene} ready."
done

log_info "ETH3D setup complete."
log_info "Data directory: ${DATASET_DIR}"
echo ""
echo "Available scenes:"
for scene in "${SCENE_ARRAY[@]}"; do
    scene_dir="${DATASET_DIR}/${scene}"
    if [[ -d "$scene_dir" ]]; then
        n_images=$(find "$scene_dir" -name "*.JPG" -o -name "*.jpg" -o -name "*.png" 2>/dev/null | wc -l)
        has_json="no"
        [[ -f "${scene_dir}/cameras.json" ]] && has_json="yes"
        echo "  ${scene}: ${n_images} images, cameras.json: ${has_json}"
    fi
done
