#!/bin/bash

# LineExtraction Local Development Environment Setup Script
# This script sets up the development environment without Docker
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REMOVE_MODE=false

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

show_help() {
    cat << EOF
LineExtraction Local Development Environment Setup

Usage: $0 [OPTIONS]

OPTIONS:
    --remove, -r    Remove the development environment instead of installing
    --help, -h      Show this help message

DESCRIPTION:
    This script sets up (or removes) the development environment for the
    LineExtraction project. It must be run with sudo privileges.

EXAMPLES:
    sudo $0             # Install development environment
    sudo $0 --remove    # Remove development environment
EOF
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --remove|-r)
                REMOVE_MODE=true
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                writeError "Unknown option: $1. Use --help for usage information."
                ;;
        esac
    done
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

# Check if running as root/sudo
check_privileges() {
    if [[ $EUID -ne 0 ]]; then
        writeError "This script must be run as root (use sudo)."
    fi
}

# Main function
main() {
    parse_args "$@"
    check_privileges

    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Starting LineExtraction local development environment removal..."
    else
        writeInfo "Starting LineExtraction local development environment setup..."
    fi

    # Ensure we're in the project root
    PROJECT_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
    cd "${PROJECT_ROOT}"

    check_os

    # Use modular scripts with appropriate flag
    local flag=""
    if [[ "$REMOVE_MODE" == "true" ]]; then
        flag="--remove"
        # Remove in reverse order for dependencies
        "${SCRIPT_DIR}/setup-precommit.sh" $flag
        "${SCRIPT_DIR}/install-clangd.sh" $flag
        "${SCRIPT_DIR}/install-bazel.sh" $flag
        "${SCRIPT_DIR}/install-uv.sh" $flag
        "${SCRIPT_DIR}/install-system-packages.sh" $flag

        # Manually remove Python environment
        if [[ -d ".venv" ]]; then
            writeInfo "Removing Python virtual environment..."
            rm -rf .venv
            writeInfo "Python virtual environment removed."
        fi
    else
        # Install in normal order
        "${SCRIPT_DIR}/install-system-packages.sh" $flag
        "${SCRIPT_DIR}/install-uv.sh" $flag
        "${SCRIPT_DIR}/install-bazel.sh" $flag
        "${SCRIPT_DIR}/install-clangd.sh" $flag
        "${SCRIPT_DIR}/setup-python-env.sh" $flag
        "${SCRIPT_DIR}/setup-precommit.sh" $flag
    fi

    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Environment removal completed successfully!"
    else
        writeInfo "Setup completed successfully!"
        writeInfo "You can now build the project using:"
        writeInfo "  For CMake: mkdir build && cd build && cmake .. && cmake --build . -j\$(nproc)"
        writeInfo "  For Bazel: bazel build //..."
        writeInfo ""
        writeInfo "To activate the Python environment: source .venv/bin/activate"
    fi
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
