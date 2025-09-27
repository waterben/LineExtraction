#!/bin/bash

# Setup/remove Python environment using UV
set -euo pipefail

writeError() {
    echo -e "\e[0;31mERROR: $1\e[0m" >&2
    exit 1
}

writeInfo() {
    echo -e "\e[0;32mINFO: $1\e[0m"
}

setup_python_env() {
    local venv_path="${1:-.venv}"
    local project_root="${2:-$(pwd)}"

    writeInfo "Setting up Python environment..."

    # Change to project root
    cd "${project_root}"

    if [[ ! -f "pyproject.toml" ]]; then
        writeError "pyproject.toml not found in ${project_root}. Please run this script from the project root or specify the correct path."
    fi

    # Ensure UV is available
    if ! command -v uv >/dev/null 2>&1; then
        writeError "UV is not installed. Please install UV first."
    fi

    # Check if uv.lock exists and is valid, create if needed
    local need_lock=false
    if [[ ! -f "uv.lock" ]]; then
        need_lock=true
    elif [[ ! -s "uv.lock" ]]; then
        need_lock=true
    elif ! grep -q "^version" "uv.lock" 2>/dev/null; then
        need_lock=true
    fi

    if [[ "$need_lock" == "true" ]]; then
        writeInfo "Creating/updating uv.lock from pyproject.toml..."
        # Remove corrupted or empty uv.lock file first
        rm -f uv.lock
        uv lock
    fi

    # Create virtual environment
    if [[ ! -d "${venv_path}" ]]; then
        writeInfo "Creating virtual environment at ${venv_path}..."
        uv venv "${venv_path}"
    fi

    # Install dependencies
    writeInfo "Installing Python dependencies..."
    source "${venv_path}/bin/activate"
    uv sync --locked

    writeInfo "Python environment set up successfully at ${venv_path}."
    writeInfo "To activate: source ${venv_path}/bin/activate"
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    setup_python_env "$@"
fi
