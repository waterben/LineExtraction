#!/bin/bash
# ==============================================================================
# Bazel Feature Detection Script
# ==============================================================================
# Detects available system features (Qt5, OpenGL, CUDA, etc.) and generates
# a .bazelrc.user file with appropriate build flags.
#
# Usage:
#   ./tools/scripts/detect_bazel_features.sh [--force]
#
# Options:
#   --force    Overwrite existing .bazelrc.user
#
# Generated file: .bazelrc.user (gitignored)
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUTPUT_FILE="${PROJECT_ROOT}/.bazelrc.user"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Feature detection results
HAVE_QT5=false
HAVE_OPENGL=false
HAVE_CUDA=false
HAVE_PHOTO=false

# Qt5 version (if found)
QT5_VERSION=""

# ==============================================================================
# Utility functions
# ==============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# ==============================================================================
# Feature detection functions
# ==============================================================================

detect_qt5() {
    log_info "Checking for Qt5..."

    # Method 1: pkg-config
    if command -v pkg-config &>/dev/null; then
        if pkg-config --exists Qt5Core Qt5Widgets Qt5Gui 2>/dev/null; then
            QT5_VERSION=$(pkg-config --modversion Qt5Core 2>/dev/null || echo "unknown")
            HAVE_QT5=true
            log_success "Qt5 found via pkg-config (version: ${QT5_VERSION})"
            return 0
        fi
    fi

    # Method 2: qmake
    if command -v qmake &>/dev/null; then
        local qmake_version
        qmake_version=$(qmake -query QT_VERSION 2>/dev/null || echo "")
        if [[ -n "${qmake_version}" && "${qmake_version}" == 5.* ]]; then
            QT5_VERSION="${qmake_version}"
            HAVE_QT5=true
            log_success "Qt5 found via qmake (version: ${QT5_VERSION})"
            return 0
        fi
    fi

    # Method 3: Check common installation paths
    local qt5_paths=(
        "/usr/include/qt5"
        "/usr/include/x86_64-linux-gnu/qt5"
        "/usr/local/include/qt5"
        "/opt/Qt5"
    )

    for path in "${qt5_paths[@]}"; do
        if [[ -d "${path}" ]]; then
            HAVE_QT5=true
            QT5_VERSION="unknown (found at ${path})"
            log_success "Qt5 found at ${path}"
            return 0
        fi
    done

    log_warning "Qt5 not found"
    return 1
}

detect_opengl() {
    log_info "Checking for OpenGL..."

    # Check for OpenGL headers
    local gl_headers=(
        "/usr/include/GL/gl.h"
        "/usr/include/GL/glu.h"
    )

    local found_headers=true
    for header in "${gl_headers[@]}"; do
        if [[ ! -f "${header}" ]]; then
            found_headers=false
            break
        fi
    done

    # Check for OpenGL libraries
    local gl_libs=(
        "libGL.so"
        "libGLU.so"
    )

    local found_libs=true
    for lib in "${gl_libs[@]}"; do
        if ! ldconfig -p 2>/dev/null | grep -q "${lib}"; then
            # Try direct file check as fallback
            if [[ ! -f "/usr/lib/x86_64-linux-gnu/${lib}" && ! -f "/usr/lib/${lib}" ]]; then
                found_libs=false
                break
            fi
        fi
    done

    if [[ "${found_headers}" == "true" && "${found_libs}" == "true" ]]; then
        HAVE_OPENGL=true
        log_success "OpenGL found (GL + GLU)"
        return 0
    fi

    # Alternative: Check via pkg-config
    if command -v pkg-config &>/dev/null; then
        if pkg-config --exists gl glu 2>/dev/null; then
            HAVE_OPENGL=true
            log_success "OpenGL found via pkg-config"
            return 0
        fi
    fi

    log_warning "OpenGL not found (or incomplete installation)"
    return 1
}

detect_cuda() {
    log_info "Checking for CUDA..."

    # Check for nvcc compiler
    if command -v nvcc &>/dev/null; then
        local cuda_version
        cuda_version=$(nvcc --version 2>/dev/null | grep "release" | sed 's/.*release \([0-9.]*\).*/\1/')
        if [[ -n "${cuda_version}" ]]; then
            HAVE_CUDA=true
            log_success "CUDA found (version: ${cuda_version})"
            return 0
        fi
    fi

    # Check CUDA_HOME or CUDA_PATH
    local cuda_paths=(
        "${CUDA_HOME:-}"
        "${CUDA_PATH:-}"
        "/usr/local/cuda"
        "/opt/cuda"
    )

    for path in "${cuda_paths[@]}"; do
        if [[ -n "${path}" && -d "${path}" && -f "${path}/bin/nvcc" ]]; then
            HAVE_CUDA=true
            log_success "CUDA found at ${path}"
            return 0
        fi
    done

    # Check for NVIDIA driver (GPU present but no toolkit)
    if command -v nvidia-smi &>/dev/null; then
        log_warning "NVIDIA GPU detected but CUDA toolkit not found"
    else
        log_warning "CUDA not found"
    fi

    return 1
}

detect_opencv_photo() {
    log_info "Checking for OpenCV photo module..."

    # This is typically not available in BCR OpenCV, but check anyway
    if command -v pkg-config &>/dev/null; then
        if pkg-config --exists opencv4 2>/dev/null; then
            # Check if photo module is available
            local opencv_libs
            opencv_libs=$(pkg-config --libs opencv4 2>/dev/null || echo "")
            if echo "${opencv_libs}" | grep -q "photo"; then
                HAVE_PHOTO=true
                log_success "OpenCV photo module found"
                return 0
            fi
        fi
    fi

    # Note: BCR OpenCV typically doesn't include photo module
    log_warning "OpenCV photo module not available (expected - BCR OpenCV doesn't include it)"
    return 1
}

# ==============================================================================
# Main script
# ==============================================================================

main() {
    local force=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --force|-f)
                force=true
                shift
                ;;
            --help|-h)
                echo "Usage: $0 [--force]"
                echo ""
                echo "Detects available features and generates .bazelrc.user"
                echo ""
                echo "Options:"
                echo "  --force, -f    Overwrite existing .bazelrc.user"
                echo "  --help, -h     Show this help"
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           Bazel Feature Detection for LineExtraction          ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""

    # Check if output file exists
    if [[ -f "${OUTPUT_FILE}" && "${force}" != "true" ]]; then
        log_warning ".bazelrc.user already exists. Use --force to overwrite."
        echo ""
        echo "Current contents:"
        echo "─────────────────"
        cat "${OUTPUT_FILE}"
        echo "─────────────────"
        exit 0
    fi

    # Run all detections
    echo "Detecting available features..."
    echo ""

    detect_qt5 || true
    detect_opengl || true
    detect_cuda || true
    detect_opencv_photo || true

    echo ""
    echo "═══════════════════════════════════════════════════════════════════"
    echo "Detection Summary"
    echo "═══════════════════════════════════════════════════════════════════"
    echo ""

    printf "  %-20s %s\n" "Qt5:" "$([[ ${HAVE_QT5} == true ]] && echo -e "${GREEN}Available${NC}" || echo -e "${YELLOW}Not found${NC}")"
    printf "  %-20s %s\n" "OpenGL:" "$([[ ${HAVE_OPENGL} == true ]] && echo -e "${GREEN}Available${NC}" || echo -e "${YELLOW}Not found${NC}")"
    printf "  %-20s %s\n" "CUDA:" "$([[ ${HAVE_CUDA} == true ]] && echo -e "${GREEN}Available${NC}" || echo -e "${YELLOW}Not found${NC}")"
    printf "  %-20s %s\n" "OpenCV Photo:" "$([[ ${HAVE_PHOTO} == true ]] && echo -e "${GREEN}Available${NC}" || echo -e "${YELLOW}Not found${NC}")"

    echo ""

    # Generate .bazelrc.user
    log_info "Generating ${OUTPUT_FILE}..."

    cat > "${OUTPUT_FILE}" << EOF
# ==============================================================================
# Auto-generated Bazel user configuration
# ==============================================================================
# Generated by: tools/scripts/detect_bazel_features.sh
# Generated at: $(date -Iseconds)
#
# This file is gitignored and contains local feature flags based on detected
# system capabilities. Re-run the detection script to regenerate:
#
#   ./tools/scripts/detect_bazel_features.sh --force
#
# You can also manually override these settings below.
# ==============================================================================

# ┌──────────────────────────────────────────────────────────────────────────┐
# │ Feature Flags (auto-detected)                                             │
# └──────────────────────────────────────────────────────────────────────────┘

# Qt5 Support (GUI applications like line_analyzer)
# Detected: ${HAVE_QT5}$(if [[ ${HAVE_QT5} == true && -n "${QT5_VERSION}" ]]; then echo " (version: ${QT5_VERSION})"; fi)
build --//bazel:enable_qt5=${HAVE_QT5}

# OpenGL Support (3D visualization)
# Detected: ${HAVE_OPENGL}
build --//bazel:enable_opengl=${HAVE_OPENGL}

# CUDA Support (GPU acceleration)
# Detected: ${HAVE_CUDA}
build --//bazel:enable_cuda=${HAVE_CUDA}

# OpenCV Photo Module (advanced image processing)
# Detected: ${HAVE_PHOTO}
# Note: BCR OpenCV typically doesn't include this module
build --//bazel:enable_photo=${HAVE_PHOTO}

# ┌──────────────────────────────────────────────────────────────────────────┐
# │ User Customizations (add your settings below)                            │
# └──────────────────────────────────────────────────────────────────────────┘

# Example: Override compiler
# build --repo_env=CC=clang
# build --repo_env=CXX=clang++

# Example: Custom output base
# startup --output_base=/path/to/faster/disk

# Example: Additional jobs
# build --jobs=16

EOF

    log_success "Generated ${OUTPUT_FILE}"

    echo ""
    echo "═══════════════════════════════════════════════════════════════════"
    echo "Next Steps"
    echo "═══════════════════════════════════════════════════════════════════"
    echo ""
    echo "  The following features are now enabled by default:"
    echo ""

    if [[ ${HAVE_QT5} == true ]]; then
        echo "    • Qt5 applications (e.g., bazel build //apps/line_analyzer:app_line_analyzer)"
    fi
    if [[ ${HAVE_OPENGL} == true ]]; then
        echo "    • OpenGL-based visualization"
    fi
    if [[ ${HAVE_CUDA} == true ]]; then
        echo "    • CUDA GPU acceleration"
    fi

    echo ""
    echo "  You can now build without specifying feature flags:"
    echo ""
    echo "    bazel build //..."
    echo "    bazel test //..."
    echo ""

    if [[ ${HAVE_QT5} == true ]]; then
        echo "  To run the line analyzer:"
        echo ""
        echo "    bazel run //apps/line_analyzer:app_line_analyzer"
        echo ""
    fi

    echo "  To modify settings, edit: ${OUTPUT_FILE}"
    echo ""
}

main "$@"
