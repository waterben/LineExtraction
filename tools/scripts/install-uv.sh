#!/bin/bash

# Install UV (Python package manager)
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

install_uv() {
    local install_dir="${1:-/usr/local/bin}"
    local user_install="${2:-false}"
    
    writeInfo "Installing UV (Python package manager) version ${UV_VERSION}..."
    
    if command -v uv >/dev/null 2>&1; then
        writeWarning "UV is already installed. Skipping..."
        return
    fi
    
    if [[ "$user_install" == "true" ]]; then
        # User installation (for local setup)
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.cargo/bin:$PATH"
    else
        # System installation (for Docker)
        curl -Ls "https://github.com/astral-sh/uv/releases/download/${UV_VERSION}/uv-x86_64-unknown-linux-musl.tar.gz" | tar xfz - --strip-components=1 -C "${install_dir}"
        chmod 755 "${install_dir}/uv" "${install_dir}/uvx"
    fi
    
    if command -v uv >/dev/null 2>&1; then
        writeInfo "UV installed successfully. Version: $(uv --version)"
    else
        writeError "UV installation failed."
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # Default to user installation for local setup
    install_uv "/usr/local/bin" "true" "$@"
fi
