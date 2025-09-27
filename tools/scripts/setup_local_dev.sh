#!/bin/bash

# LineExtraction Local Development Environment Setup Script
# This script sets up the development environment without Docker
set -euo pipefail

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

# Install system packages
install_system_packages() {
    writeInfo "Installing system packages..."

    sudo apt update

    # Base packages from docker/base/common_packages.txt
    sudo apt install -y \
        sudo ca-certificates curl cmake doxygen fontconfig moreutils jq \
        build-essential llvm clang gcc g++ git git-lfs python3 python3-venv \
        clang-format clang-tidy ccache lcov locales zstd libarchive-tools \
        shellcheck acl python3-dev xxd

    # Development packages from docker/devenv/common_packages.txt
    sudo apt install -y vim less gdb nano libarchive-tools zsh

    writeInfo "System packages installed successfully."
}

# Get the directory of this script to find the docker scripts
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_SCRIPTS_DIR="${SCRIPT_DIR}/../../docker/scripts"

# Install development tools using docker script
install_dev_tools() {
    writeInfo "Installing development tools..."

    # Set environment variables for local installation
    export INSTALL_DIR="/usr/local/bin"

    # Check if tools are already installed
    local tools_to_check=("uv" "bazel" "buildifier" "yq" "gh" "actionlint" "ruff")
    local missing_tools=()

    for tool in "${tools_to_check[@]}"; do
        if ! command -v "$tool" >/dev/null 2>&1; then
            missing_tools+=("$tool")
        fi
    done

    if [[ ${#missing_tools[@]} -eq 0 ]]; then
        writeWarning "All development tools are already installed. Skipping..."
        return
    fi

    writeInfo "Installing missing tools: ${missing_tools[*]}"

    # Run the install script with sudo for system-wide installation
    sudo -E "$DOCKER_SCRIPTS_DIR/install_base_tools"

    writeInfo "Development tools installed successfully."
}

# Install development environment tools
install_devenv_tools() {
    writeInfo "Installing development environment tools..."

    if command -v clangd >/dev/null 2>&1; then
        writeWarning "clangd is already installed. Skipping..."
        return
    fi

    # Set environment variables for local installation
    export INSTALL_DIR="/usr/local"

    # Run the install script with sudo for system-wide installation
    sudo -E "$DOCKER_SCRIPTS_DIR/install_devenv_tools"

    if command -v clangd >/dev/null 2>&1; then
        writeInfo "clangd installed successfully. Version: $(clangd --version | head -n1)"
    else
        writeError "clangd installation failed."
    fi
}

# Setup Python environment for local development
setup_python_env_local() {
    writeInfo "Setting up local Python environment..."

    if [[ ! -f "pyproject.toml" ]]; then
        writeError "pyproject.toml not found. Please run this script from the project root."
    fi

    # For local development, we use a simpler setup with .venv
    # The docker script is designed for system-wide multi-version setup

    # Create virtual environment
    if [[ ! -d ".venv" ]]; then
        uv venv .venv
        writeInfo "Created .venv virtual environment"
    else
        writeInfo ".venv virtual environment already exists"
    fi

    # Install dependencies
    source .venv/bin/activate
    uv sync --locked
    deactivate

    writeInfo "Local Python environment set up successfully."
}

# Install pre-commit hooks
setup_precommit() {
    writeInfo "Setting up pre-commit hooks..."

    if [[ ! -f ".pre-commit-config.yaml" ]]; then
        writeWarning ".pre-commit-config.yaml not found. Skipping pre-commit setup."
        return
    fi

    source .venv/bin/activate || writeError "Python virtual environment not found. Please run setup_python_env first."

    pre-commit install --config .pre-commit-config.yaml

    writeInfo "Pre-commit hooks installed successfully."
}

# Main function
main() {
    writeInfo "Starting LineExtraction local development environment setup..."

    check_os
    install_system_packages
    install_dev_tools
    install_devenv_tools
    setup_python_env_local
    setup_precommit

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
