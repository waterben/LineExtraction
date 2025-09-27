#!/bin/bash

# Install/remove Bazel/Bazelisk
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
    writeInfo "Installing Bazel version ${BAZELISK_VERSION}..."

    if command -v bazel >/dev/null 2>&1; then
        writeWarning "Bazel is already installed. Skipping..."
        return
    fi

    # Download and install bazelisk
    curl -Ls "https://github.com/bazelbuild/bazelisk/releases/download/v${BAZELISK_VERSION}/bazelisk-linux-amd64" -o /tmp/bazelisk
    mv /tmp/bazelisk "/usr/local/bin/bazel"
    chmod +x "/usr/local/bin/bazel"

    # Create bazelisk symlink for compatibility
    ln -sf "/usr/local/bin/bazel" "/usr/local/bin/bazelisk" 2>/dev/null || true

    if command -v bazel >/dev/null 2>&1; then
        writeInfo "Bazel installed successfully. Version: $(bazel version 2>/dev/null | head -n1 || echo 'Bazel installed')"
    else
        writeError "Bazel installation failed."
    fi
}

remove_bazel() {
    writeInfo "Removing Bazel/Bazelisk..."

    if ! command -v bazel >/dev/null 2>&1; then
        writeWarning "Bazel is not installed. Skipping..."
        return
    fi

    # Clean Bazel cache
    if command -v bazel >/dev/null 2>&1; then
        bazel clean --expunge 2>/dev/null || true
    fi

    # Remove binaries
    rm -f /usr/local/bin/bazel /usr/local/bin/bazelisk

    # Remove user cache directories
    rm -rf ~/.cache/bazel* ~/.cache/bazelisk 2>/dev/null || true

    writeInfo "Bazel removed successfully."
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
        remove_bazel
    else
        install_bazel
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
