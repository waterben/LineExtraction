#!/bin/bash

# Install Bazel/Bazelisk
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "${SCRIPT_DIR}/tool-versions.sh"

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

install_bazel() {
    local install_dir="${1:-/usr/local/bin}"
    
    writeInfo "Installing Bazel version ${BAZELISK_VERSION}..."
    
    if command -v bazel >/dev/null 2>&1; then
        writeWarning "Bazel is already installed. Skipping..."
        return
    fi
    
    # Download and install bazelisk
    curl -Ls "https://github.com/bazelbuild/bazelisk/releases/download/v${BAZELISK_VERSION}/bazelisk-linux-amd64" -o /tmp/bazelisk
    
    if [[ "$(id -u)" -ne 0 ]] && [[ "$install_dir" == "/usr/local/bin" ]]; then
        sudo mv /tmp/bazelisk "${install_dir}/bazel"
        sudo chmod +x "${install_dir}/bazel"
    else
        mv /tmp/bazelisk "${install_dir}/bazel"
        chmod +x "${install_dir}/bazel"
    fi
    
    # Create bazelisk symlink for compatibility
    if [[ "$(id -u)" -ne 0 ]] && [[ "$install_dir" == "/usr/local/bin" ]]; then
        sudo ln -sf "${install_dir}/bazel" "${install_dir}/bazelisk" 2>/dev/null || true
    else
        ln -sf "${install_dir}/bazel" "${install_dir}/bazelisk" 2>/dev/null || true
    fi
    
    if command -v bazel >/dev/null 2>&1; then
        writeInfo "Bazel installed successfully. Version: $(bazel version 2>/dev/null | head -n1 || echo 'Bazel installed')"
    else
        writeError "Bazel installation failed."
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_bazel "$@"
fi
