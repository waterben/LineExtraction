#!/bin/bash

# LineExtraction Local Development Environment Setup Script
# This script sets up or removes the development environment without Docker
# It reuses scripts from docker/scripts for consistency
#
# USAGE:
#   This script can be run from anywhere within the LineExtraction project.
#   It automatically detects the project root and navigates there.
#
#   Installation (requires sudo):
#     sudo ./tools/scripts/setup_local_dev.sh
#
#   VS Code extensions only (no sudo):
#     ./tools/scripts/setup_local_dev.sh --only-extensions
#
#   Removal:
#     sudo ./tools/scripts/setup_local_dev.sh --remove-tools
#     sudo ./tools/scripts/setup_local_dev.sh --remove-packages
#
# WHAT IT INSTALLS:
#   - System packages (build-essential, cmake, git, etc.)
#   - Development tools (uv, bazel, clangd, gh, yq, ruff, actionlint)
#   - Python virtual environment (.venv) with project dependencies
#   - Git-aware shell prompt with persistent history
#   - Pre-commit hooks
#   - VS Code extensions from devcontainer.json
#
# SMART INSTALLATION:
#   - The script checks before installing and skips already installed components
#   - Safe to run multiple times - only missing components will be installed
#   - VS Code extensions are never removed (including GitHub Copilot)
#   - Python environment with >10 packages is considered configured
#
# ENVIRONMENT SETUP:
#   - Creates ~/.vscode_profile with colorized git-aware prompt
#   - Configures persistent command history in ~/.cmd_history/
#   - Auto-activates Python venv in new shells
#   - WSL: Configures DISPLAY for X server (GUI/OpenGL support)
#
# IMPORTANT NOTES:
#   - Requires sudo for system operations (install/remove packages and tools)
#   - No sudo needed for: --help, --only-extensions
#   - Safe to run multiple times (idempotent with intelligent skip logic)
#   - Removal operations are selective (use flags as needed)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_SCRIPTS_DIR="${SCRIPT_DIR}/../../docker/scripts"
DOCKER_BASE_DIR="${SCRIPT_DIR}/../../docker/base"
DOCKER_DEVENV_DIR="${SCRIPT_DIR}/../../docker/devenv"
REMOVE_TOOLS_MODE=false
REMOVE_PACKAGES_MODE=false
ONLY_EXTENSIONS_MODE=false

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
    cat << HELP_EOF
LineExtraction Local Development Environment Setup

Usage: $0 [OPTIONS]

OPTIONS:
    --remove-tools, --remove, -r  Remove only development tools and Python environments
    --remove-packages             Remove only APT packages from Docker configuration
    --only-extensions             Install only VS Code extensions (no sudo required)
    --help, -h                    Show this help message

DESCRIPTION:
    This script sets up (or removes) the development environment for the
    LineExtraction project. It reuses scripts from docker/scripts for consistency.
    Most operations require sudo privileges, except --only-extensions.

EXAMPLES:
    sudo $0                     # Install full development environment
    sudo $0 --remove-tools      # Remove only tools (uv, bazel, clangd, etc.)
    sudo $0 --remove-packages   # Remove only APT packages
    $0 --only-extensions        # Install VS Code extensions only (no sudo)

NOTE:
    This script installs system-wide packages and tools. Package removal with
    --remove-packages will only remove common packages, NOT system packages
    (system_packages.txt) for system stability.
HELP_EOF
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --remove-tools)
                REMOVE_TOOLS_MODE=true
                shift
                ;;
            --remove-packages)
                REMOVE_PACKAGES_MODE=true
                shift
                ;;
            --remove|-r)
                REMOVE_TOOLS_MODE=true
                shift
                ;;
            --only-extensions)
                ONLY_EXTENSIONS_MODE=true
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

# Setup user context (works with or without sudo)
setup_user_context() {
    if [[ -n "${SUDO_USER:-}" ]]; then
        # Running under sudo
        ORIGINAL_USER="$SUDO_USER"
        ORIGINAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
    else
        # Running as regular user
        ORIGINAL_USER="$USER"
        ORIGINAL_HOME="$HOME"
    fi

    # Verify user home directory exists
    if [[ ! -d "$ORIGINAL_HOME" ]]; then
        writeError "User home directory not found: $ORIGINAL_HOME"
    fi
}

# Check if running as root/sudo (for operations requiring root)
check_privileges() {
    if [[ $EUID -ne 0 ]]; then
        writeError "This operation requires root privileges. Please run with sudo."
    fi

    setup_user_context
    writeInfo "Running system operations as root, user operations as: $ORIGINAL_USER"
}

# Run command as original user (works with or without sudo)
run_as_user() {
    if [[ $EUID -eq 0 ]] && [[ -n "${SUDO_USER:-}" ]] && [[ "$SUDO_USER" != "root" ]]; then
        # Running as root via sudo - switch to original user
        sudo -u "$SUDO_USER" -H "$@"
    else
        # Running as regular user - just execute
        "$@"
    fi
}

# Install system packages using Docker script
install_system_packages() {
    writeInfo "Installing system packages using Docker configuration..."

    # Verify required scripts exist
    if [[ ! -x "$DOCKER_SCRIPTS_DIR/install_apt_packages_from_list" ]]; then
        writeError "Docker script not found or not executable: $DOCKER_SCRIPTS_DIR/install_apt_packages_from_list"
    fi

    # Update package lists
    writeInfo "Updating package lists..."
    apt-get update

    # Install packages from base system_packages.txt
    if [[ -f "$DOCKER_BASE_DIR/system_packages.txt" ]]; then
        writeInfo "Installing base system packages from Docker configuration..."
        "$DOCKER_SCRIPTS_DIR/install_apt_packages_from_list" --file "$DOCKER_BASE_DIR/system_packages.txt"
    else
        writeError "Docker base system packages file not found: $DOCKER_BASE_DIR/system_packages.txt"
    fi

    # Install packages from base common_packages.txt
    if [[ -f "$DOCKER_BASE_DIR/common_packages.txt" ]]; then
        writeInfo "Installing base common packages from Docker configuration..."
        "$DOCKER_SCRIPTS_DIR/install_apt_packages_from_list" --file "$DOCKER_BASE_DIR/common_packages.txt"
    else
        writeError "Docker base common packages file not found: $DOCKER_BASE_DIR/common_packages.txt"
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

    # Make environment variables persistent
    writeInfo "Making environment variables persistent..."
    if ! grep -q "REQUESTS_CA_BUNDLE" /etc/environment 2>/dev/null; then
        echo "export REQUESTS_CA_BUNDLE=/etc/ssl/certs/ca-certificates.crt" >> /etc/environment
    fi
    if ! grep -q "BAZELISK_BASE_URL" /etc/environment 2>/dev/null; then
        echo "export BAZELISK_BASE_URL=\"https://github.com/bazelbuild/bazel/releases/download\"" >> /etc/environment
    fi

    # Clean up package cache
    writeInfo "Cleaning up package cache..."
    rm -rf /var/lib/apt/lists/*

    writeInfo "System packages installed successfully."
}

# Install development tools using Docker script
install_base_tools() {
    writeInfo "Installing base development tools using Docker script..."

    # Verify required script exists
    if [[ ! -x "$DOCKER_SCRIPTS_DIR/install_base_tools" ]]; then
        writeError "Docker script not found or not executable: $DOCKER_SCRIPTS_DIR/install_base_tools"
    fi

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
        writeInfo "All base development tools are already installed. Skipping..."
        return
    fi

    writeInfo "Installing missing tools: ${missing_tools[*]}"

    # Run the Docker install script
    "$DOCKER_SCRIPTS_DIR/install_base_tools"

    writeInfo "Base development tools installed successfully."
}

# Install development environment tools using Docker script
install_devenv_tools() {
    writeInfo "Installing development environment tools using Docker script..."

    # Verify required script exists
    if [[ ! -x "$DOCKER_SCRIPTS_DIR/install_devenv_tools" ]]; then
        writeError "Docker script not found or not executable: $DOCKER_SCRIPTS_DIR/install_devenv_tools"
    fi

    if command -v clangd >/dev/null 2>&1; then
        writeInfo "clangd is already installed. Version: $(clangd --version | head -n1)"
        writeInfo "Skipping clangd installation..."
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
    writeInfo "Setting up local Python environment..."

    # Check for pyproject.toml
    if [[ ! -f "pyproject.toml" ]]; then
        writeError "pyproject.toml not found. Please run this script from the project root."
    fi

    # Check if uv is available
    if ! command -v uv >/dev/null 2>&1; then
        writeError "uv is not installed or not in PATH. Please ensure install_base_tools completed successfully."
    fi

    # Check if Python environment is already properly set up
    if [[ -d ".venv" ]] && [[ -f ".venv/bin/activate" ]] && [[ -f "uv.lock" ]]; then
        writeInfo "Python virtual environment already exists."

        # Quick check if dependencies are up to date
        # Count installed packages in site-packages
        pkg_count=$(run_as_user bash -c 'ls .venv/lib/python*/site-packages/*.dist-info 2>/dev/null | wc -l')

        if [[ $pkg_count -gt 10 ]]; then
            writeInfo "  Found $pkg_count packages installed"
            writeInfo "  Python environment already properly configured. Skipping..."
            writeInfo "  To force reinstall, remove .venv directory first: rm -rf .venv"
            return
        fi
    fi

    # For local development, we use a simpler setup with .venv
    # The docker script is designed for system-wide multi-version setup

    # Run as original user to ensure correct ownership
    run_as_user bash -c '
        # Detect required Python version from pyproject.toml
        REQUIRED_PYTHON=$(grep "requires-python" pyproject.toml | sed -E "s/.*>=([0-9]+\.[0-9]+).*/\1/")
        if [[ -z "$REQUIRED_PYTHON" ]]; then
            REQUIRED_PYTHON="3.8"
            echo "Could not detect required Python version, defaulting to $REQUIRED_PYTHON"
        else
            echo "Detected required Python version: >=$REQUIRED_PYTHON"
        fi

        # Create/update uv.lock from pyproject.toml if needed
        if [[ ! -f "uv.lock" ]] || [[ "pyproject.toml" -nt "uv.lock" ]]; then
            echo "Creating/updating uv.lock from pyproject.toml..."
            uv lock
        fi

        # Create virtual environment with the correct Python version
        if [[ ! -d ".venv" ]]; then
            echo "Creating .venv virtual environment with Python $REQUIRED_PYTHON..."
            uv venv .venv --python "$REQUIRED_PYTHON"
            echo "Created .venv virtual environment"
        else
            echo ".venv virtual environment already exists"
            # Verify Python version in existing venv
            VENV_PYTHON_VERSION=$(.venv/bin/python --version 2>&1 | grep -oP "Python \K[0-9]+\.[0-9]+")
            echo "Existing venv uses Python $VENV_PYTHON_VERSION"
        fi

        # Install dependencies
        echo "Installing dependencies with uv sync..."
        uv sync --locked

        # Verify installation
        if [[ -f ".venv/bin/activate" ]]; then
            source .venv/bin/activate
            echo "Python environment verification:"
            echo "  Python version: $(python --version)"
            echo "  Python path: $(which python)"
            echo "  Pip version: $(pip --version)"
            echo "  Installed packages: $(pip list | wc -l) packages"

            # Verify key packages are installed
            echo ""
            echo "Verifying key packages:"
            for pkg in pre-commit ruff clang-format yamllint; do
                if command -v "$pkg" >/dev/null 2>&1; then
                    echo "  ✓ $pkg installed"
                else
                    echo "  ✗ $pkg NOT found"
                fi
            done

            deactivate
        fi
    '

    writeInfo "Local Python environment set up successfully."
    writeInfo "To activate: source .venv/bin/activate"
}

# Setup local development environment profile
setup_dev_profile() {
    writeInfo "Setting up local development environment profile..."

    # Check if profile is already set up
    if [[ -f "$ORIGINAL_HOME/.vscode_profile" ]] && grep -q "source ~/.vscode_profile" "$ORIGINAL_HOME/.bashrc" 2>/dev/null; then
        writeInfo "Development profile already configured. Skipping..."
        return
    fi

    # Run as original user
    run_as_user bash -c '
        # Create development folders
        echo "Creating development folders..."
        mkdir -p ~/.cache ~/.ccache ~/.ssh ~/.cmd_history

        # Create .inputrc for history search
        echo "Setting up .inputrc for command history search..."
        echo "$""include /etc/inputrc" > ~/.inputrc
        echo "# Map \"page up\" and \"page down\" to search the history" >> ~/.inputrc
        echo "\"\\e[5~\": history-search-backward" >> ~/.inputrc
        echo "\"\\e[6~\": history-search-forward" >> ~/.inputrc

        # Create .vscode_profile with git support for PS1
        echo "Creating .vscode_profile..."
        echo "# Use local copy of the PS1 support scripts provided by git" > ~/.vscode_profile
        echo "source /usr/local/bin/git-prompt.sh" >> ~/.vscode_profile
        echo "# Enable color codes" >> ~/.vscode_profile
        echo "export COLOR_RED=\"\\[\\033[1;31m\\]\"" >> ~/.vscode_profile
        echo "export COLOR_GREEN=\"\\[\\033[1;32m\\]\"" >> ~/.vscode_profile
        echo "export COLOR_BLUE=\"\\[\\033[1;34m\\]\"" >> ~/.vscode_profile
        echo "export COLOR_END=\"\\[\\033[0m\\]\"" >> ~/.vscode_profile
        echo "# Change the prompt accordingly" >> ~/.vscode_profile
        echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> ~/.vscode_profile
        echo "export GIT_PS1_SHOWSTASHSTATE=0" >> ~/.vscode_profile
        echo "export GIT_PS1_SHOWUNTRACKEDFILES=0" >> ~/.vscode_profile
        echo "export GIT_PS1_SHOWUPSTREAM=\"auto\"" >> ~/.vscode_profile
        echo "export PS1=\"\${COLOR_GREEN}\\u@\\h\${COLOR_END}:\${COLOR_BLUE}\\w\${COLOR_RED}\$(__git_ps1)\${COLOR_END}\\n> \"" >> ~/.vscode_profile
        echo "# ccache uses default ~/.ccache directory" >> ~/.vscode_profile
        echo "# Set the bash history file to be persistent and ensure it is activated before prompt" >> ~/.vscode_profile
        echo "export PROMPT_COMMAND=\"history -a\" && export HISTFILE=\"\${HOME}/.cmd_history/.bash_history\"" >> ~/.vscode_profile

        # Create history file
        touch ~/.cmd_history/.bash_history

        # Activate Python venv if it exists
        echo "# Activate Python virtual environment if it exists" >> ~/.vscode_profile
        echo "if [[ -f \"\${HOME}/.venv/bin/activate\" ]]; then" >> ~/.vscode_profile
        echo "    source \"\${HOME}/.venv/bin/activate\"" >> ~/.vscode_profile
        echo "elif [[ -f \".venv/bin/activate\" ]]; then" >> ~/.vscode_profile
        echo "    source .venv/bin/activate" >> ~/.vscode_profile
        echo "fi" >> ~/.vscode_profile

        # WSL-specific: Set DISPLAY for X server support
        echo "# WSL: Configure DISPLAY for X server (GUI/OpenGL support)" >> ~/.vscode_profile
        echo "if grep -qi microsoft /proc/version 2>/dev/null; then" >> ~/.vscode_profile
        echo "    # WSL 2: Use Windows host IP from resolv.conf" >> ~/.vscode_profile
        echo "    export DISPLAY=\$(cat /etc/resolv.conf | grep nameserver | awk '\''{print \$2}'\''):0" >> ~/.vscode_profile
        echo "    # Optional: Uncomment if using WSLg (WSL 2 with built-in GUI support)" >> ~/.vscode_profile
        echo "    # export DISPLAY=:0" >> ~/.vscode_profile
        echo "fi" >> ~/.vscode_profile

        # Enable environment in bashrc (only if not already present)
        if ! grep -q "source ~/.vscode_profile" ~/.bashrc 2>/dev/null; then
            echo "Adding .vscode_profile to .bashrc..."
            echo "" >> ~/.bashrc
            echo "# BEGIN - Appended via LineExtraction setup_local_dev.sh" >> ~/.bashrc
            echo "source ~/.vscode_profile" >> ~/.bashrc
            echo "# END - Appended via LineExtraction setup_local_dev.sh" >> ~/.bashrc
        else
            echo ".vscode_profile already sourced in .bashrc"
        fi
    '

    writeInfo "Development environment profile setup successfully."
}

# Install pre-commit hooks
setup_precommit() {
    writeInfo "Setting up pre-commit hooks..."

    if [[ ! -f ".pre-commit-config.yaml" ]]; then
        writeWarning ".pre-commit-config.yaml not found. Skipping pre-commit setup."
        return
    fi

    # Check if pre-commit hooks are already installed
    if [[ -f ".git/hooks/pre-commit" ]] && grep -q "pre-commit" .git/hooks/pre-commit 2>/dev/null; then
        writeInfo "Pre-commit hooks already installed. Skipping..."
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

        # Check if pre-commit is available
        if ! command -v pre-commit >/dev/null 2>&1; then
            echo "WARNING: pre-commit not found in venv. Installing..."
            pip install pre-commit
        fi

        pre-commit install --config .pre-commit-config.yaml
    '

    writeInfo "Pre-commit hooks installed successfully."
}

# Remove all installed tools using remove_tools script
remove_all_tools() {
    writeInfo "Removing all development tools and environments..."

    # Verify required script exists
    if [[ ! -x "$DOCKER_SCRIPTS_DIR/remove_tools" ]]; then
        writeError "Docker script not found or not executable: $DOCKER_SCRIPTS_DIR/remove_tools"
    fi

    # Set environment variables for local installation paths
    export INSTALL_DIR="/usr/local/bin"
    export INSTALL_BASE_DIR="/usr/local"

    # Run the Docker remove script
    "$DOCKER_SCRIPTS_DIR/remove_tools"

    writeInfo "All development tools and environments removed successfully."
}

# Remove local Python environment
remove_python_env_local() {
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
}

# Remove local development profile
remove_dev_profile() {
    writeInfo "Removing local development environment profile..."

    run_as_user bash -c '
        # Remove .vscode_profile entry from .bashrc
        if [[ -f ~/.bashrc ]] && grep -q "source ~/.vscode_profile" ~/.bashrc 2>/dev/null; then
            echo "Removing .vscode_profile from .bashrc..."
            # Remove the entire block added by setup script
            sed -i "/# BEGIN - Appended via LineExtraction setup_local_dev.sh/,/# END - Appended via LineExtraction setup_local_dev.sh/d" ~/.bashrc
        fi

        # Remove profile files
        if [[ -f ~/.vscode_profile ]]; then
            rm -f ~/.vscode_profile
            echo "Removed .vscode_profile"
        fi

        if [[ -f ~/.inputrc ]]; then
            # Only remove if it was created by our script (contains our history search config)
            if grep -q "history-search-backward" ~/.inputrc 2>/dev/null; then
                rm -f ~/.inputrc
                echo "Removed .inputrc"
            fi
        fi

        # Optionally remove cmd_history (commented out to preserve history)
        # if [[ -d ~/.cmd_history ]]; then
        #     rm -rf ~/.cmd_history
        #     echo "Removed .cmd_history directory"
        # fi
    '

    writeInfo "Development environment profile removed successfully."
}

# Remove pre-commit hooks
remove_precommit() {
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
}

# Install VS Code extensions from devcontainer.json
install_vscode_extensions() {
    writeInfo "Installing VS Code extensions from devcontainer.json..."
    writeInfo "NOTE: Existing extensions (including GitHub Copilot) will NOT be removed."

    run_as_user bash -c '
        DEVCONTAINER_JSON=".devcontainer/devcontainer.json"

        if [[ ! -f "$DEVCONTAINER_JSON" ]]; then
            echo "WARNING: devcontainer.json not found at $DEVCONTAINER_JSON"
            echo "Skipping VS Code extension installation."
            exit 0
        fi

        # Check if code command is available
        if ! command -v code >/dev/null 2>&1; then
            echo "WARNING: VS Code (code command) not found in PATH."
            echo "Skipping extension installation. Install VS Code first:"
            echo "  https://code.visualstudio.com/docs/setup/linux"
            exit 0
        fi

        # Extract extensions using grep and sed (no jq dependency)
        # Look for lines between "extensions": [ and ]
        extensions=$(sed -n "/\"extensions\":/,/]/p" "$DEVCONTAINER_JSON" | \
                     grep -E "\"[^\"]+\.[^\"]+\"" | \
                     sed -E "s/.*\"([^\"]+)\".*/\1/")

        if [[ -z "$extensions" ]]; then
            echo "No extensions found in devcontainer.json"
            exit 0
        fi

        total_count=$(echo "$extensions" | wc -l)
        echo "Found $total_count extensions to install"
        echo ""

        # Install each extension
        installed=0
        skipped=0
        failed=0

        while IFS= read -r ext; do
            [[ -z "$ext" ]] && continue

            # Check if extension is already installed
            # Filter out header line and check for exact match
            if code --list-extensions 2>/dev/null | grep -v "^Extensions installed" | grep -qx "$ext"; then
                echo "[SKIP] $ext (already installed)"
                ((skipped++))
                continue
            fi

            echo "[INSTALL] $ext"
            if code --install-extension "$ext" --force >/dev/null 2>&1; then
                ((installed++))
            else
                echo "  [ERROR] Failed to install $ext"
                ((failed++))
            fi
        done <<< "$extensions"

        echo ""
        echo "Extension installation summary:"
        echo "  ✓ Newly installed: $installed"
        if [[ $skipped -gt 0 ]]; then
            echo "  ⊚ Already installed: $skipped"
        fi
        if [[ $failed -gt 0 ]]; then
            echo "  ✗ Failed: $failed"
        fi
        echo "  Total: $total_count"
    '

    writeInfo "VS Code extensions installation completed."
}

# Remove APT packages from Docker configuration files
remove_system_packages() {
    writeInfo "Removing APT packages from Docker configuration..."

    # Function to extract package names from config file (same logic as install script)
    extract_package_names() {
        local file_path="$1"
        if [[ ! -f "$file_path" ]]; then
            return
        fi
        # sed 's/`.*`//g' removes everything between backticks (comments)
        # tr '\n' ' ' converts newlines to spaces
        cat "$file_path" | sed 's/`.*`//g' | tr '\n' ' '
    }

    # Get packages from base configuration (only common_packages, NOT system_packages)
    local base_packages=""
    if [[ -f "$DOCKER_BASE_DIR/common_packages.txt" ]]; then
        base_packages=$(extract_package_names "$DOCKER_BASE_DIR/common_packages.txt")
        writeInfo "Base common packages to remove: $base_packages"
    fi

    # Note: system_packages.txt is NOT included in removal for system stability

    # Get packages from devenv configuration
    local devenv_packages=""
    if [[ -f "$DOCKER_DEVENV_DIR/common_packages.txt" ]]; then
        devenv_packages=$(extract_package_names "$DOCKER_DEVENV_DIR/common_packages.txt")
        writeInfo "DevEnv packages to remove: $devenv_packages"
    fi

    # Combine all packages
    local all_packages="$base_packages $devenv_packages"

    if [[ -z "$all_packages" ]]; then
        writeWarning "No packages found to remove."
        return
    fi

    # Remove packages (only if they are installed)
    writeInfo "Removing APT packages..."
    local packages_to_remove=""
    # No explicit exclusions needed since system_packages.txt is not processed for removal

    for package in $all_packages; do
        # Skip empty strings
        [[ -z "$package" ]] && continue

        # All packages in the removal list are safe to remove since system_packages.txt is excluded

        # Check if package is installed
        if dpkg -l "$package" >/dev/null 2>&1; then
            packages_to_remove="$packages_to_remove $package"
        else
            writeInfo "Package $package is not installed, skipping..."
        fi
    done

    if [[ -n "$packages_to_remove" ]]; then
        writeInfo "Removing installed packages:$packages_to_remove"
        # Use --auto-remove to also remove dependencies that are no longer needed
        DEBIAN_FRONTEND=noninteractive apt-get remove --auto-remove -y $packages_to_remove

        # Clean up
        apt-get autoremove -y
        apt-get autoclean

        writeInfo "APT packages removed successfully."
    else
        writeInfo "No packages need to be removed."
    fi

    # Remove environment variables
    writeInfo "Cleaning up environment variables..."
    if [[ -f "/etc/environment" ]]; then
        # Create a backup
        cp /etc/environment /etc/environment.bak.$(date +%Y%m%d_%H%M%S)

        # Remove our specific environment variables
        if grep -q "REQUESTS_CA_BUNDLE" /etc/environment; then
            writeInfo "Removing REQUESTS_CA_BUNDLE from /etc/environment"
            sed -i '/^export REQUESTS_CA_BUNDLE=/d' /etc/environment
        fi

        if grep -q "BAZELISK_BASE_URL" /etc/environment; then
            writeInfo "Removing BAZELISK_BASE_URL from /etc/environment"
            sed -i '/^export BAZELISK_BASE_URL=/d' /etc/environment
        fi
    fi
}

# Global variables for user handling
ORIGINAL_USER=""
ORIGINAL_HOME=""

# Main function
main() {
    parse_args "$@"

    # Setup user context (works with or without sudo)
    setup_user_context

    # Ensure we're in the project root
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
    if [[ ! -f "${PROJECT_ROOT}/pyproject.toml" ]] || [[ ! -d "${PROJECT_ROOT}/.git" ]]; then
        writeError "Project root detection failed. Expected pyproject.toml and .git in: $PROJECT_ROOT"
    fi
    cd "${PROJECT_ROOT}"
    writeInfo "Working directory: $PROJECT_ROOT"

    check_os

    # Handle --only-extensions mode (no sudo required)
    if [[ "$ONLY_EXTENSIONS_MODE" == "true" ]]; then
        writeInfo "Installing VS Code extensions only (no sudo required)..."
        install_vscode_extensions
        writeInfo "VS Code extensions installation completed."
        return 0
    fi

    # Check if sudo is required for the requested operation
    local needs_sudo=false
    if [[ "$REMOVE_TOOLS_MODE" == "true" ]] || [[ "$REMOVE_PACKAGES_MODE" == "true" ]]; then
        needs_sudo=true
    fi

    # For installation mode, check if system packages will be installed
    if [[ "$REMOVE_TOOLS_MODE" == "false" ]] && [[ "$REMOVE_PACKAGES_MODE" == "false" ]] && [[ "$ONLY_EXTENSIONS_MODE" == "false" ]]; then
        needs_sudo=true
    fi

    # Verify sudo privileges for operations that need them
    if [[ "$needs_sudo" == "true" ]]; then
        check_privileges
    fi

    # Determine mode and show appropriate message
    if [[ "$REMOVE_TOOLS_MODE" == "true" ]] || [[ "$REMOVE_PACKAGES_MODE" == "true" ]]; then
        local mode_desc=""
        if [[ "$REMOVE_TOOLS_MODE" == "true" ]] && [[ "$REMOVE_PACKAGES_MODE" == "true" ]]; then
            mode_desc="tools and packages"
        elif [[ "$REMOVE_TOOLS_MODE" == "true" ]]; then
            mode_desc="tools only"
        else
            mode_desc="packages only"
        fi
        writeInfo "Starting LineExtraction local development environment removal ($mode_desc)..."

        # Handle removal operations
        if [[ "$REMOVE_TOOLS_MODE" == "true" ]]; then
            writeInfo "Removing development tools and environments..."

            # Remove in reverse order for dependencies - user components first
            remove_precommit
            remove_dev_profile
            remove_python_env_local

            # Then system components as root
            remove_all_tools

            writeInfo "Development tools removal completed."
        fi

        if [[ "$REMOVE_PACKAGES_MODE" == "true" ]]; then
            writeInfo "Removing APT packages..."
            remove_system_packages
            writeInfo "APT packages removal completed."
        fi

        writeInfo ""
        writeInfo "=========================================="
        writeInfo "Environment removal completed successfully!"
        writeInfo "=========================================="
        writeInfo ""

        if [[ "$REMOVE_TOOLS_MODE" == "true" ]] && [[ "$REMOVE_PACKAGES_MODE" == "false" ]]; then
            writeInfo "NOTE: APT packages were NOT removed. Use --remove-packages to remove them."
        elif [[ "$REMOVE_PACKAGES_MODE" == "true" ]] && [[ "$REMOVE_TOOLS_MODE" == "false" ]]; then
            writeInfo "NOTE: Development tools were NOT removed. Use --remove-tools to remove them."
        fi
    else
        # Installation mode
        writeInfo "Starting LineExtraction local development environment setup..."
        writeInfo "This will install system packages and development tools."

        # Verify Docker scripts exist
        writeInfo "Verifying Docker scripts..."
        local required_scripts=(
            "$DOCKER_SCRIPTS_DIR/install_apt_packages_from_list"
            "$DOCKER_SCRIPTS_DIR/install_base_tools"
            "$DOCKER_SCRIPTS_DIR/install_devenv_tools"
        )
        for script in "${required_scripts[@]}"; do
            if [[ ! -x "$script" ]]; then
                writeError "Required Docker script not found or not executable: $script"
            fi
        done

        # Verify Docker configuration files exist
        local required_files=(
            "$DOCKER_BASE_DIR/system_packages.txt"
            "$DOCKER_BASE_DIR/common_packages.txt"
            "$DOCKER_DEVENV_DIR/common_packages.txt"
        )
        for file in "${required_files[@]}"; do
            if [[ ! -f "$file" ]]; then
                writeError "Required Docker configuration file not found: $file"
            fi
        done

        writeInfo "All required Docker scripts and configuration files found."

        # Install in correct order - system components first, then user components
        install_system_packages
        install_base_tools
        install_devenv_tools

        # User-specific components as original user
        setup_python_env_local
        setup_dev_profile
        setup_precommit
        install_vscode_extensions

        writeInfo ""
        writeInfo "=========================================="
        writeInfo "Setup completed successfully!"
        writeInfo "=========================================="
        writeInfo ""
        writeInfo "Next steps:"
        writeInfo "  1. Reload your shell or run: source ~/.bashrc"
        writeInfo "  2. The Python virtual environment will activate automatically"
        writeInfo "  3. Open VS Code: code ."
        writeInfo ""
        writeInfo "Your shell now has:"
        writeInfo "  ✓ Git-aware colorized prompt"
        writeInfo "  ✓ Persistent command history (~/.cmd_history/.bash_history)"
        writeInfo "  ✓ Page-up/page-down for history search"
        writeInfo "  ✓ Auto-activation of Python venv"
        if grep -qi microsoft /proc/version 2>/dev/null; then
            writeInfo "  ✓ DISPLAY configured for X server (WSL)"
        fi
        writeInfo ""
        writeInfo "You can now build the project using:"
        writeInfo "  For CMake: mkdir -p build && cd build && cmake .. && cmake --build . -j\$(nproc)"
        writeInfo "  For Bazel: bazel build //..."
        writeInfo ""
        writeInfo "Python environment: .venv (auto-activated)"
        writeInfo "  - Activate manually: source .venv/bin/activate"
        writeInfo "  - Python packages: $(run_as_user bash -c 'source .venv/bin/activate 2>/dev/null && pip list | wc -l' || echo 'N/A') installed"
        writeInfo ""
        writeInfo "Available tools:"
        writeInfo "  - uv (Python package manager)"
        writeInfo "  - bazel/bazelisk (Build system)"
        writeInfo "  - buildifier (Bazel formatter)"
        writeInfo "  - clangd (C++ language server)"
        writeInfo "  - gh (GitHub CLI)"
        writeInfo "  - yq (YAML processor)"
        writeInfo "  - actionlint (GitHub Actions linter)"
        writeInfo "  - ruff (Python linter/formatter)"
        writeInfo ""
        if grep -qi microsoft /proc/version 2>/dev/null; then
            writeInfo "WSL Detected:"
            writeInfo "  For GUI/OpenGL applications, ensure you have an X server running on Windows"
            writeInfo "  (e.g., VcXsrv, X410). See docs/WSL_SETUP.md for details."
            writeInfo ""
        fi
    fi  # End of else block (install mode)
}  # End of main function

# Check if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
