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

# Install UV (Python package manager)
install_uv() {
    writeInfo "Installing UV (Python package manager)..."
    
    if command -v uv >/dev/null 2>&1; then
        writeWarning "UV is already installed. Skipping..."
        return
    fi
    
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.cargo/bin:$PATH"
    
    if command -v uv >/dev/null 2>&1; then
        writeInfo "UV installed successfully. Version: $(uv --version)"
    else
        writeError "UV installation failed."
    fi
}

# Install Bazel
install_bazel() {
    writeInfo "Installing Bazel..."
    
    if command -v bazel >/dev/null 2>&1; then
        writeWarning "Bazel is already installed. Skipping..."
        return
    fi
    
    local bazel_version="1.26.0"
    curl -Ls "https://github.com/bazelbuild/bazelisk/releases/download/v${bazel_version}/bazelisk-linux-amd64" -o /tmp/bazelisk
    sudo mv /tmp/bazelisk /usr/local/bin/bazel
    sudo chmod +x /usr/local/bin/bazel
    
    if command -v bazel >/dev/null 2>&1; then
        writeInfo "Bazel installed successfully. Version: $(bazel version 2>/dev/null | head -n1 || echo 'Bazel installed')"
    else
        writeError "Bazel installation failed."
    fi
}

# Install clangd
install_clangd() {
    writeInfo "Installing clangd..."
    
    if command -v clangd >/dev/null 2>&1; then
        writeWarning "clangd is already installed. Skipping..."
        return
    fi
    
    local clangd_version="21.1.0"
    curl -Ls "https://github.com/clangd/clangd/releases/download/${clangd_version}/clangd-linux-${clangd_version}.zip" | sudo bsdtar xf - --strip-components=1 -C /usr/local
    
    if command -v clangd >/dev/null 2>&1; then
        writeInfo "clangd installed successfully. Version: $(clangd --version | head -n1)"
    else
        writeError "clangd installation failed."
    fi
}

# Setup Python environment
setup_python_env() {
    writeInfo "Setting up Python environment..."
    
    if [[ ! -f "pyproject.toml" ]]; then
        writeError "pyproject.toml not found. Please run this script from the project root."
    fi
    
    # Create virtual environment
    if [[ ! -d ".venv" ]]; then
        uv venv .venv
    fi
    
    # Install dependencies
    source .venv/bin/activate
    uv sync --locked
    
    writeInfo "Python environment set up successfully."
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
    install_uv
    install_bazel
    install_clangd
    setup_python_env
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
