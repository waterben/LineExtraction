#!/bin/bash

# Install system packages using the existing Docker package lists
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DOCKER_DIR="${SCRIPT_DIR}/../../docker"

writeError() {
    echo -e "\e[0;31mERROR: $1\e[0m" >&2
    exit 1
}

writeInfo() {
    echo -e "\e[0;32mINFO: $1\e[0m"
}

install_system_packages() {
    writeInfo "Installing system packages..."
    
    # Check if we can use the Docker script
    if [[ -x "${DOCKER_DIR}/scripts/install_apt_packages_from_list" ]]; then
        # Use the Docker script with sudo if not root
        if [[ "$(id -u)" -ne 0 ]]; then
            sudo apt update
            sudo "${DOCKER_DIR}/scripts/install_apt_packages_from_list" -f "${DOCKER_DIR}/base/common_packages.txt"
            sudo "${DOCKER_DIR}/scripts/install_apt_packages_from_list" -f "${DOCKER_DIR}/devenv/common_packages.txt"
        else
            apt update
            "${DOCKER_DIR}/scripts/install_apt_packages_from_list" -f "${DOCKER_DIR}/base/common_packages.txt"
            "${DOCKER_DIR}/scripts/install_apt_packages_from_list" -f "${DOCKER_DIR}/devenv/common_packages.txt"
        fi
    else
        # Fallback to direct apt install
        writeInfo "Docker script not found, using fallback method..."
        sudo apt update
        
        # Install base packages
        sudo apt install -y \
            sudo ca-certificates curl cmake doxygen fontconfig moreutils jq \
            build-essential llvm clang gcc g++ git git-lfs python3 python3-venv \
            clang-format clang-tidy ccache lcov locales zstd libarchive-tools \
            shellcheck acl python3-dev xxd
        
        # Install development packages
        sudo apt install -y vim less gdb nano libarchive-tools zsh
    fi
    
    writeInfo "System packages installed successfully."
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_system_packages "$@"
fi
