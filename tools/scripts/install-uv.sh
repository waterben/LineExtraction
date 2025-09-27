#!/bin/bash

# Install/remove UV (Python package manager)
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
    writeInfo "Installing UV (Python package manager) version ${UV_VERSION}..."

    if command -v uv >/dev/null 2>&1; then
        writeWarning "UV is already installed. Skipping..."
        return
    fi

    # System installation
    curl -Ls "https://github.com/astral-sh/uv/releases/download/${UV_VERSION}/uv-x86_64-unknown-linux-musl.tar.gz" | tar xfz - --strip-components=1 -C "/usr/local/bin"
    chmod 755 "/usr/local/bin/uv" "/usr/local/bin/uvx"

    if command -v uv >/dev/null 2>&1; then
        writeInfo "UV installed successfully. Version: $(uv --version)"
    else
        writeError "UV installation failed."
    fi
}

remove_uv() {
    writeInfo "Removing UV (Python package manager)..."

    if ! command -v uv >/dev/null 2>&1; then
        writeWarning "UV is not installed. Skipping..."
        return
    fi

    # Remove UV binaries
    rm -f /usr/local/bin/uv /usr/local/bin/uvx

    # Also check user installations
    if [[ -f "$HOME/.cargo/bin/uv" ]]; then
        rm -f "$HOME/.cargo/bin/uv" "$HOME/.cargo/bin/uvx"
    fi

    writeInfo "UV removed successfully."
}

# Main function
main() {
    local remove_mode=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --remove)
                remove_mode=true
                shift
                ;;
            *)
                shift
                ;;
        esac
    done

    if [[ "$remove_mode" == "true" ]]; then
        remove_uv
    else
        install_uv
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
