#!/bin/bash

# LineExtraction Local Development Environment Setup Script
# This script sets up the development environment without Docker
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

writeError() {
    echo -e "\e[0;31mERROR: $1\e[0m" >&2
    exit 1
}

writeWarning() {
    echo -e "\e[0;33mWARNING: $1\e[0m"
}

writeInfo() {
    echo -e "\e[0;32mINFO: $1\e[0m"
}

# Check if running on supported OS
check_os() {
    if [[ ! -f /etc/os-release ]]; then
        writeError "Cannot detect OS. This script supports Ubuntu/Debian systems."
    fi
    
    source /etc/os-release
    if [[ "$ID" != "ubuntu" && "$ID" != "debian" ]]; then
        writeError "Unsupported OS: $ID. This script supports Ubuntu/Debian systems."
    fi
    
    writeInfo "Detected OS: $ID $VERSION_ID"
}

# Main function
main() {
    writeInfo "Starting LineExtraction local development environment setup..."
    
    # Ensure we're in the project root
    PROJECT_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
    cd "${PROJECT_ROOT}"
    
    check_os
    
    # Use modular scripts
    "${SCRIPT_DIR}/install-system-packages.sh"
    "${SCRIPT_DIR}/install-uv.sh"
    "${SCRIPT_DIR}/install-bazel.sh"
    "${SCRIPT_DIR}/install-clangd.sh"
    "${SCRIPT_DIR}/setup-python-env.sh"
    "${SCRIPT_DIR}/setup-precommit.sh"
    
    writeInfo "Setup completed successfully!"
    writeInfo "You can now build the project using:"
    writeInfo "  For CMake: mkdir build && cd build && cmake .. && cmake --build . -j\$(nproc)"
    writeInfo "  For Bazel: bazel build //..."
    writeInfo ""
    writeInfo "To activate the Python environment: source .venv/bin/activate"
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
