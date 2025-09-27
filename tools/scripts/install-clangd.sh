#!/bin/bash

# Install/remove clangd
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
    writeInfo "Installing clangd version ${CLANGD_VERSION}..."

    if command -v clangd >/dev/null 2>&1; then
        writeWarning "clangd is already installed. Skipping..."
        return
    fi

    # Install clangd and indexing tools
    curl -Ls "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd-linux-${CLANGD_VERSION}.zip" | bsdtar xf - --strip-components=1 -C "/usr/local"
    curl -Ls "https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd_indexing_tools-linux-${CLANGD_VERSION}.zip" | bsdtar xf - --strip-components=1 -C "/usr/local"
    chmod 755 "/usr/local/bin/clangd"*

    if command -v clangd >/dev/null 2>&1; then
        writeInfo "clangd installed successfully. Version: $(clangd --version | head -n1)"
    else
        writeError "clangd installation failed."
    fi
}

remove_clangd() {
    writeInfo "Removing clangd..."

    if ! command -v clangd >/dev/null 2>&1; then
        writeWarning "clangd is not installed. Skipping..."
        return
    fi

    # Remove clangd binaries
    rm -f /usr/local/bin/clangd*

    # Remove clangd related directories
    rm -rf /usr/local/lib/clang* /usr/local/include/clang* 2>/dev/null || true

    writeInfo "clangd removed successfully."
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
        remove_clangd
    else
        install_clangd
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
