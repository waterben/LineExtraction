#!/bin/bash

# Setup pre-commit hooks
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

setup_precommit() {
    local project_root="${1:-$(pwd)}"
    local config_file="${2:-.pre-commit-config.yaml}"
    
    writeInfo "Setting up pre-commit hooks..."
    
    # Change to project root
    cd "${project_root}"
    
    if [[ ! -f "${config_file}" ]]; then
        writeWarning "${config_file} not found in ${project_root}. Skipping pre-commit setup."
        return
    fi
    
    # Check if we're in a virtual environment or if pre-commit is available globally
    if ! command -v pre-commit >/dev/null 2>&1; then
        # Try to activate .venv if it exists
        if [[ -f ".venv/bin/activate" ]]; then
            writeInfo "Activating virtual environment..."
            source .venv/bin/activate
        else
            writeError "pre-commit not found and no virtual environment available. Please install pre-commit or set up Python environment first."
        fi
    fi
    
    # Install pre-commit hooks
    pre-commit install --config "${config_file}"
    
    writeInfo "Pre-commit hooks installed successfully."
}

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    setup_precommit "$@"
fi
