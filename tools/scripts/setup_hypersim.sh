#!/bin/bash
# =============================================================================
# setup_hypersim.sh - Download Hypersim sample scenes for 3D line evaluation
# =============================================================================
#
# Downloads a curated subset of Apple's Hypersim dataset (synthetic indoor
# scenes with known camera intrinsics, depth, and surface normals).  Uses
# HTTP range requests to selectively extract only the needed files (preview
# images + camera metadata) from the large scene ZIP archives, avoiding
# multi-GB full downloads.
#
# Source: https://github.com/apple/ml-hypersim
# Reference: Roberts et al., "Hypersim: A Photorealistic Synthetic Dataset
#   for Holistic Indoor Scene Understanding", ICCV 2021.
#
# Requires: Python 3 with requests, numpy (and optionally h5py for camera
#   metadata).  The project .venv is used automatically.
#
# Usage:
#   ./tools/scripts/setup_hypersim.sh [OPTIONS]
#
# Options:
#   --scenes LIST   Comma-separated scene names (default: ai_001_001,ai_001_002)
#   --max-frames N  Max frames per scene (default: 20)
#   --clean         Remove existing data before download
#   --help          Show this help message
#
# Output:
#   resources/datasets/Hypersim/<scene>/images/       - RGB images (tone-mapped)
#   resources/datasets/Hypersim/<scene>/depth/        - Depth maps (.npy)
#   resources/datasets/Hypersim/<scene>/cameras.json  - Per-frame camera params
#
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DATASET_DIR="${WORKSPACE_ROOT}/resources/datasets/Hypersim"
VENV_PYTHON="${WORKSPACE_ROOT}/.venv/bin/python"

# Defaults
SCENES="ai_001_001,ai_001_002"
MAX_FRAMES=20
CLEAN=false

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
    head -33 "$0" | tail -28
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --scenes)
            SCENES="$2"
            shift 2
            ;;
        --max-frames)
            MAX_FRAMES="$2"
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

if [[ ! -x "$VENV_PYTHON" ]]; then
    log_error "Python venv not found at ${VENV_PYTHON}"
    log_error "Run ./tools/scripts/setup_local_dev.sh first."
    exit 1
fi

# Check required Python packages; install h5py if missing
"$VENV_PYTHON" -c "import requests" 2>/dev/null || {
    log_error "Python 'requests' package not found in venv."
    log_error "Install it:  ${VENV_PYTHON} -m pip install requests"
    exit 1
}

if ! "$VENV_PYTHON" -c "import h5py" 2>/dev/null; then
    log_warn "h5py not found in venv — installing..."
    if command -v uv &> /dev/null; then
        uv pip install --quiet h5py
    else
        "$VENV_PYTHON" -m pip install --quiet h5py
    fi
fi

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

if [[ "$CLEAN" == true ]] && [[ -d "$DATASET_DIR" ]]; then
    log_warn "Cleaning existing Hypersim data..."
    rm -rf "$DATASET_DIR"
fi

mkdir -p "$DATASET_DIR"

IFS=',' read -ra SCENE_ARRAY <<< "$SCENES"

for scene in "${SCENE_ARRAY[@]}"; do
    scene_dir="${DATASET_DIR}/${scene}"
    cam_file="${scene_dir}/cameras.json"

    if [[ -f "$cam_file" ]]; then
        existing_frames=$("$VENV_PYTHON" -c "
import json, sys
with open('${cam_file}') as f:
    d = json.load(f)
print(len(d.get('frames', [])))
" 2>/dev/null || echo "0")
        if [[ "$existing_frames" -gt 0 ]]; then
            log_info "Scene ${scene} already has ${existing_frames} frames, skipping."
            continue
        fi
        log_warn "Scene ${scene} has empty cameras.json, re-downloading."
        rm -rf "$scene_dir"
    fi

    log_step "Processing scene: ${scene}"
    mkdir -p "${scene_dir}/images" "${scene_dir}/depth"

    # Use Python to selectively download from the remote ZIP via range requests
    "$VENV_PYTHON" - "$scene" "$scene_dir" "$MAX_FRAMES" <<'PYEOF'
"""Selectively extract Hypersim scene data from remote ZIP via HTTP range requests.

Downloads only tone-mapped preview images plus camera HDF5 metadata from the
Apple CDN ZIP archives, avoiding multi-GB full scene downloads.
"""

import io
import json
import os
import re
import sys
import zipfile

import numpy as np
import requests

# Increase read size for faster streaming extraction.
zipfile.ZipExtFile.MIN_READ_SIZE = 2 ** 20

# Apple CDN base URL (confirmed to support Accept-Ranges: bytes).
CDN_BASE = (
    "https://docs-assets.developer.apple.com"
    "/ml-research/datasets/hypersim/v1/scenes"
)

# Known Hypersim intrinsics (shared by every scene).
INTRINSIC_K = [[886.81, 0.0, 512.0], [0.0, 886.81, 384.0], [0.0, 0.0, 1.0]]
IMG_WIDTH = 1024
IMG_HEIGHT = 768


class WebFile:
    """File-like wrapper around a remote URL using HTTP range requests.

    Allows ``zipfile.ZipFile`` to read the central directory and selectively
    extract entries without downloading the entire archive.  Inspired by the
    approach in ``github.com/apple/ml-hypersim/contrib/99991/download.py``.
    """

    def __init__(self, url: str, session: requests.Session) -> None:
        with session.head(url, allow_redirects=True) as resp:
            resp.raise_for_status()
            self.size = int(resp.headers["content-length"])
        self.url = url
        self.session = session
        self.offset = 0

    # -- file-like interface required by zipfile.ZipFile --

    def seekable(self) -> bool:
        return True

    def tell(self) -> int:
        return self.offset

    def seek(self, offset: int, whence: int = 0) -> int:
        if whence == 0:
            self.offset = offset
        elif whence == 1:
            self.offset = min(self.offset + offset, self.size)
        elif whence == 2:
            self.offset = max(0, self.size + offset)
        return self.offset

    def read(self, n: int | None = None) -> bytes:
        available = self.size - self.offset
        if n is None:
            n = available
        else:
            n = min(n, available)
        if n <= 0:
            return b""
        end_inclusive = self.offset + n - 1
        headers = {"Range": f"bytes={self.offset}-{end_inclusive}"}
        with self.session.get(self.url, headers=headers) as resp:
            resp.raise_for_status()
            data = resp.content
        self.offset += len(data)
        return data


def _frame_index(filename: str) -> int | None:
    """Extract zero-based frame index from a Hypersim filename."""
    m = re.search(r"frame\.(\d+)\.", filename)
    return int(m.group(1)) if m else None


def download_scene(scene: str, scene_dir: str, max_frames: int) -> int:
    """Download preview images + camera metadata for *scene*.

    Returns the number of image frames successfully extracted.
    """
    url = f"{CDN_BASE}/{scene}.zip"
    session = requests.Session()

    print(f"  Opening remote ZIP: {url}")
    try:
        wf = WebFile(url, session)
    except Exception as exc:
        print(f"  ERROR: Cannot access ZIP ({exc})", file=sys.stderr)
        return 0

    zf = zipfile.ZipFile(wf)

    # Classify entries we care about.
    cam_pos_entry = None
    cam_rot_entry = None
    preview_entries: list[tuple[int, zipfile.ZipInfo]] = []
    depth_entries: list[tuple[int, zipfile.ZipInfo]] = []

    for entry in zf.infolist():
        if entry.is_dir():
            continue
        name = entry.filename

        # Camera metadata HDF5 files.
        if name.endswith("camera_keyframe_positions.hdf5"):
            cam_pos_entry = entry
            continue
        if name.endswith("camera_keyframe_orientations.hdf5"):
            cam_rot_entry = entry
            continue

        # Tone-mapped preview images (JPEG).
        if "scene_cam_00_final_preview" in name and name.endswith(".tonemap.jpg"):
            idx = _frame_index(name)
            if idx is not None and idx < max_frames:
                preview_entries.append((idx, entry))
            continue

        # Depth maps (HDF5).
        if "scene_cam_00_geometry_hdf5" in name and name.endswith(
            ".depth_meters.hdf5"
        ):
            idx = _frame_index(name)
            if idx is not None and idx < max_frames:
                depth_entries.append((idx, entry))
            continue

    # Sort by frame index.
    preview_entries.sort(key=lambda t: t[0])
    depth_entries.sort(key=lambda t: t[0])

    # --- Extract preview images ---
    img_dir = os.path.join(scene_dir, "images")
    os.makedirs(img_dir, exist_ok=True)

    print(f"  Extracting {len(preview_entries)} preview images...")
    for idx, entry in preview_entries:
        data = zf.read(entry.filename)
        dest = os.path.join(img_dir, f"frame_{idx:04d}.jpg")
        with open(dest, "wb") as f:
            f.write(data)

    # --- Extract depth maps and convert HDF5 -> NPY ---
    depth_dir = os.path.join(scene_dir, "depth")
    os.makedirs(depth_dir, exist_ok=True)

    has_h5py = True
    try:
        import h5py
    except ImportError:
        has_h5py = False
        if depth_entries:
            print("  WARN: h5py not available — skipping depth extraction.")

    if has_h5py and depth_entries:
        print(f"  Extracting {len(depth_entries)} depth maps...")
        for idx, entry in depth_entries:
            raw = zf.read(entry.filename)
            with h5py.File(io.BytesIO(raw), "r") as hf:
                depth = np.array(hf["dataset"])
            dest = os.path.join(depth_dir, f"frame_{idx:04d}.npy")
            np.save(dest, depth)

    # --- Build cameras.json from HDF5 metadata ---
    cam_file = os.path.join(scene_dir, "cameras.json")
    frame_indices = sorted(idx for idx, _ in preview_entries)

    if cam_pos_entry and cam_rot_entry and has_h5py:
        print("  Converting camera metadata to cameras.json...")
        pos_raw = zf.read(cam_pos_entry.filename)
        rot_raw = zf.read(cam_rot_entry.filename)

        with h5py.File(io.BytesIO(pos_raw), "r") as fp:
            positions = np.array(fp["dataset"])
        with h5py.File(io.BytesIO(rot_raw), "r") as fr:
            orientations = np.array(fr["dataset"])

        frames = []
        for idx in frame_indices:
            if idx >= len(positions) or idx >= len(orientations):
                continue
            frames.append(
                {
                    "frame_id": idx,
                    "image": f"images/frame_{idx:04d}.jpg",
                    "R": orientations[idx].tolist(),
                    "t": positions[idx].tolist(),
                    "K": INTRINSIC_K,
                    "width": IMG_WIDTH,
                    "height": IMG_HEIGHT,
                }
            )

        with open(cam_file, "w") as f:
            json.dump({"scene": scene, "frames": frames}, f, indent=2)
        print(f"  Wrote cameras.json with {len(frames)} frames.")
    else:
        reason = "h5py unavailable" if not has_h5py else "camera HDF5 not found"
        print(f"  WARN: {reason} — writing stub cameras.json.")
        # Write stub with known intrinsics only.
        frames = []
        for idx in frame_indices:
            frames.append(
                {
                    "frame_id": idx,
                    "image": f"images/frame_{idx:04d}.jpg",
                    "K": INTRINSIC_K,
                    "width": IMG_WIDTH,
                    "height": IMG_HEIGHT,
                    "note": "extrinsics unavailable",
                }
            )
        with open(cam_file, "w") as f:
            json.dump({"scene": scene, "frames": frames}, f, indent=2)

    return len(preview_entries)


if __name__ == "__main__":
    scene_name = sys.argv[1]
    out_dir = sys.argv[2]
    n_frames = int(sys.argv[3])
    n = download_scene(scene_name, out_dir, n_frames)
    print(f"  Done: {n} frames extracted.")
PYEOF

    rc=$?
    if [[ $rc -ne 0 ]]; then
        log_error "  Download failed for scene ${scene}."
        continue
    fi

    log_info "Scene ${scene} ready."
done

log_info "Hypersim setup complete."
log_info "Data directory: ${DATASET_DIR}"
echo ""
echo "Available scenes:"
for scene in "${SCENE_ARRAY[@]}"; do
    scene_dir="${DATASET_DIR}/${scene}"
    if [[ -d "${scene_dir}/images" ]]; then
        n_images=$(find "${scene_dir}/images" -name "*.jpg" 2>/dev/null | wc -l)
        has_depth=$(find "${scene_dir}/depth" -name "*.npy" 2>/dev/null | wc -l)
        echo "  ${scene}: ${n_images} images, ${has_depth} depth maps"
    fi
done
