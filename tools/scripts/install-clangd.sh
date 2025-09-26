#!/bin/bash

# Install clangd
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

install_clangd() {
    local install_dir="${1:-/usr/local}"
    
    writeInfo "Installing clangd version ${CLANGD_VERSION}..."
    
    if command -v clangd >/dev/null 2>&1; then
        writeWarning "clangd is already installed. Skipping..."
        return
    fi
    
    # Install clangd and indexing tools
    if [[ "$(id -u)" -ne 0 ]] && [[ "$install_dir" == "/usr/local" ]]; then
        curl -Ls "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd-linux-${CLANGD_VERSION}.zip" | sudo bsdtar xf - --strip-components=1 -C "${install_dir}"
        curl -Ls "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd_indexing_tools-linux-${CLANGD_VERSION}.zip" | sudo bsdtar xf - --strip-components=1 -C "${install_dir}"
        sudo chmod 755 "${install_dir}/bin/clangd"*
    else
        curl -Ls "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd-linux-${CLANGD_VERSION}.zip" | bsdtar xf - --strip-components=1 -C "${install_dir}"
        curl -Ls "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd_indexing_tools-linux-${CLANGD_VERSION}.zip" | bsdtar xf - --strip-components=1 -C "${install_dir}"
        chmod 755 "${install_dir}/bin/clangd"*
    fi
    
    if command -v clangd >/dev/null 2>&1; then
        writeInfo "clangd installed successfully. Version: $(clangd --version | head -n1)"
    else
        writeError "clangd installation failed."
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    install_clangd "$@"
fi
