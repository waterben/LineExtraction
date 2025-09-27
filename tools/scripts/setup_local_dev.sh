#!/bin/bash

# LineExtraction Local Development Environment Setup Script
# This script sets up the development environment without Docker
# It reuses scripts from docker/scripts for consistency
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_SCRIPTS_DIR="${SCRIPT_DIR}/../../docker/scripts"
DOCKER_BASE_DIR="${SCRIPT_DIR}/../../docker/base"
DOCKER_DEVENV_DIR="${SCRIPT_DIR}/../../docker/devenv"
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
    LineExtraction project. It reuses scripts from docker/scripts for consistency.
    Must be run with sudo privileges.

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

    # Check if we have the original user available for user-specific operations
    if [[ -z "${SUDO_USER:-}" ]]; then
        writeWarning "SUDO_USER not set. User-specific operations will run as root."
        ORIGINAL_USER="root"
        ORIGINAL_HOME="/root"
    else
        ORIGINAL_USER="$SUDO_USER"
        ORIGINAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
    fi

    writeInfo "Running system operations as root, user operations as: $ORIGINAL_USER"
}

# Run command as original user (before sudo)
run_as_user() {
    if [[ "$ORIGINAL_USER" == "root" ]]; then
        # No original user available, run as root
        "$@"
    else
        # Run as original user with their environment
        sudo -u "$ORIGINAL_USER" -H "$@"
    fi
}

# Install system packages using Docker script
install_system_packages() {
    writeInfo "Installing system packages using Docker configuration..."

    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Removing system packages..."
        # For safety, we don't automatically remove system packages
        # Users should manually remove them if desired
        writeWarning "System packages removal not implemented for safety. Please remove manually if needed."
        return
    fi

    # Update package lists
    apt-get update

    # Install packages from base common_packages.txt
    if [[ -f "$DOCKER_BASE_DIR/common_packages.txt" ]]; then
        writeInfo "Installing base packages from Docker configuration..."
        "$DOCKER_SCRIPTS_DIR/install_apt_packages_from_list" --file "$DOCKER_BASE_DIR/common_packages.txt"
    else
        writeError "Docker base packages file not found: $DOCKER_BASE_DIR/common_packages.txt"
    fi

    # Install packages from devenv common_packages.txt
    if [[ -f "$DOCKER_DEVENV_DIR/common_packages.txt" ]]; then
        writeInfo "Installing development packages from Docker configuration..."
        "$DOCKER_SCRIPTS_DIR/install_apt_packages_from_list" --file "$DOCKER_DEVENV_DIR/common_packages.txt"
    else
        writeError "Docker devenv packages file not found: $DOCKER_DEVENV_DIR/common_packages.txt"
    fi

    # Set up locale and certificates (from Dockerfile)
    writeInfo "Setting up locale..."
    locale-gen en_US en_US.UTF-8
    update-ca-certificates

    # Set up environment variables (from Dockerfile)
    writeInfo "Setting up environment variables..."
    export REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt
    export BAZELISK_BASE_URL="https://github.com/bazelbuild/bazel/releases/download"

    # Make environment variables persistent for the session
    echo "export REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt" >> /etc/environment
    echo "export BAZELISK_BASE_URL=\"https://github.com/bazelbuild/bazel/releases/download\"" >> /etc/environment

    writeInfo "System packages installed successfully."
}

# Install development tools using Docker script
install_base_tools() {
    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Removing base development tools..."

        # Remove tools installed by install_base_tools
        local tools=("uv" "uvx" "bazel" "bazelisk" "buildifier" "yq" "gh" "actionlint" "ruff")
        for tool in "${tools[@]}"; do
            if [[ -f "/usr/local/bin/$tool" ]]; then
                rm -f "/usr/local/bin/$tool"
                writeInfo "Removed $tool"
            fi
        done

        writeInfo "Base development tools removed successfully."
        return
    fi

    writeInfo "Installing base development tools using Docker script..."

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
        writeWarning "All base development tools are already installed. Skipping..."
        return
    fi

    writeInfo "Installing missing tools: ${missing_tools[*]}"

    # Run the Docker install script
    "$DOCKER_SCRIPTS_DIR/install_base_tools"

    writeInfo "Base development tools installed successfully."
}

# Install development environment tools using Docker script
install_devenv_tools() {
    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Removing development environment tools..."

        # Remove clangd and related tools
        if [[ -f "/usr/local/bin/clangd" ]]; then
            rm -f "/usr/local/bin/clangd"
            writeInfo "Removed clangd"
        fi

        writeInfo "Development environment tools removed successfully."
        return
    fi

    writeInfo "Installing development environment tools using Docker script..."

    if command -v clangd >/dev/null 2>&1; then
        writeWarning "clangd is already installed. Skipping..."
        return
    fi

    # Set environment variables for local installation
    export INSTALL_DIR="/usr/local"

    # Run the Docker install script
    "$DOCKER_SCRIPTS_DIR/install_devenv_tools"

    if command -v clangd >/dev/null 2>&1; then
        writeInfo "clangd installed successfully. Version: $(clangd --version | head -n1)"
    else
        writeError "clangd installation failed."
    fi
}

# Setup Python environment for local development
setup_python_env_local() {
    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Removing local Python environment..."

        # Remove virtual environment and lock file as original user
        run_as_user bash -c '
            if [[ -d ".venv" ]]; then
                rm -rf .venv
                echo "Removed .venv directory"
            fi
            if [[ -f "uv.lock" ]]; then
                rm -f uv.lock
                echo "Removed uv.lock file"
            fi
        '

        writeInfo "Local Python environment removed successfully."
        return
    fi

    writeInfo "Setting up local Python environment..."

    # Check for pyproject.toml
    if [[ ! -f "pyproject.toml" ]]; then
        writeError "pyproject.toml not found. Please run this script from the project root."
    fi

    # For local development, we use a simpler setup with .venv
    # The docker script is designed for system-wide multi-version setup

    # Run as original user to ensure correct ownership
    run_as_user bash -c '
        # Create/update uv.lock from pyproject.toml if needed
        if [[ ! -f "uv.lock" ]] || [[ "pyproject.toml" -nt "uv.lock" ]]; then
            echo "Creating/updating uv.lock from pyproject.toml..."
            uv lock
        fi

        # Create virtual environment
        if [[ ! -d ".venv" ]]; then
            uv venv .venv
            echo "Created .venv virtual environment"
        else
            echo ".venv virtual environment already exists"
        fi

        # Install dependencies
        uv sync --locked
    '

    writeInfo "Local Python environment set up successfully."
    writeInfo "To activate: source .venv/bin/activate"
}

# Install pre-commit hooks
setup_precommit() {
    if [[ "$REMOVE_MODE" == "true" ]]; then
        writeInfo "Removing pre-commit hooks..."

        run_as_user bash -c '
            if [[ -f ".git/hooks/pre-commit" ]]; then
                if command -v pre-commit >/dev/null 2>&1; then
                    source .venv/bin/activate 2>/dev/null || true
                    pre-commit uninstall 2>/dev/null || true
                fi
                rm -f .git/hooks/pre-commit
                echo "Removed pre-commit hooks"
            fi
        '

        writeInfo "Pre-commit hooks removed successfully."
        return
    fi

    writeInfo "Setting up pre-commit hooks..."

    if [[ ! -f ".pre-commit-config.yaml" ]]; then
        writeWarning ".pre-commit-config.yaml not found. Skipping pre-commit setup."
        return
    fi

    # Run as original user
    run_as_user bash -c '
        # Activate virtual environment
        if [[ ! -f ".venv/bin/activate" ]]; then
            echo "ERROR: Python virtual environment not found. Please run setup_python_env first." >&2
            exit 1
        fi

        source .venv/bin/activate
        pre-commit install --config .pre-commit-config.yaml
    '

    writeInfo "Pre-commit hooks installed successfully."
}

# Global variables for user handling
ORIGINAL_USER=""
ORIGINAL_HOME=""

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
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
    cd "${PROJECT_ROOT}"

    check_os

    if [[ "$REMOVE_MODE" == "true" ]]; then
        # Remove in reverse order for dependencies - user components first
        setup_precommit
        setup_python_env_local

        # Then system components as root
        install_devenv_tools
        install_base_tools
        install_system_packages

        writeInfo "Environment removal completed successfully!"
    else
        # Install in normal order - system components first
        install_system_packages
        install_base_tools
        install_devenv_tools

        # User-specific components as original user
        setup_python_env_local
        setup_precommit

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
