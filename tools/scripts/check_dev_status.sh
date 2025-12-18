#!/bin/bash

# LineExtraction Development Environment Status Check
# This script checks the status of the local development environment setup

set -euo pipefail

COLOR_RED='\033[1;31m'
COLOR_GREEN='\033[1;32m'
COLOR_YELLOW='\033[1;33m'
COLOR_BLUE='\033[1;34m'
COLOR_END='\033[0m'

check_ok() {
    echo -e "  ${COLOR_GREEN}✓${COLOR_END} $1"
}

check_fail() {
    echo -e "  ${COLOR_RED}✗${COLOR_END} $1"
}

check_warn() {
    echo -e "  ${COLOR_YELLOW}⚠${COLOR_END} $1"
}

section_header() {
    echo -e "\n${COLOR_BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${COLOR_END}"
    echo -e "${COLOR_BLUE}$1${COLOR_END}"
    echo -e "${COLOR_BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${COLOR_END}"
}

# Check if we're in the project root
if [[ ! -f "pyproject.toml" ]] || [[ ! -d "tools/scripts" ]]; then
    echo "Error: This script must be run from the LineExtraction project root"
    exit 1
fi

section_header "System Information"
if [[ -f /etc/os-release ]]; then
    source /etc/os-release
    echo "  OS: $ID $VERSION_ID"
else
    echo "  OS: Unknown"
fi

if grep -qi microsoft /proc/version 2>/dev/null; then
    check_ok "WSL detected"
    WSL_VERSION=$(wsl.exe -l -v 2>/dev/null | grep -i ubuntu | awk '{print $3}' || echo "unknown")
    echo "  WSL Version: $WSL_VERSION"
else
    echo "  Environment: Native Linux"
fi

section_header "Development Tools"

# Check base tools
for tool in uv bazel buildifier yq gh actionlint ruff; do
    if command -v "$tool" >/dev/null 2>&1; then
        VERSION=$("$tool" --version 2>&1 | head -n1 || echo "unknown")
        check_ok "$tool ($VERSION)"
    else
        check_fail "$tool not installed"
    fi
done

# Check clangd
if command -v clangd >/dev/null 2>&1; then
    VERSION=$(clangd --version 2>&1 | head -n1 || echo "unknown")
    check_ok "clangd ($VERSION)"
else
    check_fail "clangd not installed"
fi

section_header "Python Environment"

# Check for .venv
if [[ -d ".venv" ]]; then
    check_ok "Virtual environment exists (.venv)"

    if [[ -f ".venv/bin/python" ]]; then
        PYTHON_VERSION=$(.venv/bin/python --version 2>&1)
        check_ok "Python: $PYTHON_VERSION"

        # Check key packages
        source .venv/bin/activate 2>/dev/null || true
        for pkg in pre-commit ruff yamllint clang-format; do
            if command -v "$pkg" >/dev/null 2>&1; then
                check_ok "$pkg available in venv"
            else
                check_warn "$pkg not found in venv"
            fi
        done

        PACKAGE_COUNT=$(pip list 2>/dev/null | wc -l || echo "0")
        echo "  Total packages: $PACKAGE_COUNT"
        deactivate 2>/dev/null || true
    else
        check_fail "Python interpreter not found in .venv"
    fi

    if [[ -f "uv.lock" ]]; then
        check_ok "uv.lock exists"
        if [[ "pyproject.toml" -nt "uv.lock" ]]; then
            check_warn "uv.lock is older than pyproject.toml (consider running: uv lock)"
        fi
    else
        check_warn "uv.lock missing (run: uv lock)"
    fi
else
    check_fail "Virtual environment not found (.venv)"
    echo "    Run: sudo ./tools/scripts/setup_local_dev.sh"
fi

section_header "Shell Configuration"

# Check .vscode_profile
if [[ -f "$HOME/.vscode_profile" ]]; then
    check_ok ".vscode_profile exists"

    # Check if it contains key components
    if grep -q "git-prompt.sh" "$HOME/.vscode_profile" 2>/dev/null; then
        check_ok "Git prompt configured"
    else
        check_warn "Git prompt not configured in .vscode_profile"
    fi

    if grep -q "venv/bin/activate" "$HOME/.vscode_profile" 2>/dev/null; then
        check_ok "Auto venv activation configured"
    else
        check_warn "Auto venv activation not configured"
    fi

    if grep -qi "DISPLAY" "$HOME/.vscode_profile" 2>/dev/null && grep -qi microsoft /proc/version 2>/dev/null; then
        check_ok "WSL DISPLAY configured for X server"
    fi
else
    check_fail ".vscode_profile not found"
    echo "    Run: sudo ./tools/scripts/setup_local_dev.sh"
fi

# Check .bashrc integration
if [[ -f "$HOME/.bashrc" ]] && grep -q "source ~/.vscode_profile" "$HOME/.bashrc" 2>/dev/null; then
    check_ok ".vscode_profile sourced in .bashrc"
else
    check_fail ".vscode_profile not sourced in .bashrc"
fi

# Check .inputrc
if [[ -f "$HOME/.inputrc" ]] && grep -q "history-search-backward" "$HOME/.inputrc" 2>/dev/null; then
    check_ok ".inputrc configured (page-up/down history search)"
else
    check_warn ".inputrc not configured"
fi

# Check command history
if [[ -f "$HOME/.cmd_history/.bash_history" ]]; then
    check_ok "Persistent command history configured"
else
    check_warn "Command history directory not found"
fi

section_header "Git Configuration"

# Check pre-commit hooks
if [[ -f ".git/hooks/pre-commit" ]]; then
    check_ok "Pre-commit hooks installed"

    # Check if pre-commit is functional
    if command -v pre-commit >/dev/null 2>&1; then
        source .venv/bin/activate 2>/dev/null || true
        if pre-commit --version >/dev/null 2>&1; then
            check_ok "Pre-commit is functional"
        fi
        deactivate 2>/dev/null || true
    fi
else
    check_fail "Pre-commit hooks not installed"
    echo "    Activate venv and run: pre-commit install"
fi

if [[ -f ".pre-commit-config.yaml" ]]; then
    check_ok ".pre-commit-config.yaml exists"
else
    check_warn ".pre-commit-config.yaml not found"
fi

section_header "VS Code Configuration"

if [[ -f ".vscode/settings.json" ]]; then
    check_ok ".vscode/settings.json exists"

    # Check Python interpreter setting
    if grep -q "python.defaultInterpreterPath" .vscode/settings.json 2>/dev/null; then
        PYTHON_PATH=$(grep "python.defaultInterpreterPath" .vscode/settings.json | sed -n 's/.*: *"\([^"]*\)".*/\1/p')
        if [[ "$PYTHON_PATH" == *".venv"* ]]; then
            check_ok "Python interpreter configured for local venv"
        else
            check_warn "Python interpreter: $PYTHON_PATH"
        fi
    fi
else
    check_warn ".vscode/settings.json not found"
fi

section_header "Build System"

# Check for CMake build
# if [[ -d "build" ]]; then
#     check_ok "CMake build directory exists"
#     if [[ -f "build/CMakeCache.txt" ]]; then
#         BUILD_TYPE=$(grep "CMAKE_BUILD_TYPE:" build/CMakeCache.txt | cut -d= -f2 || echo "unknown")
#         echo "  Build type: $BUILD_TYPE"
#     fi
# else
#     echo "  No CMake build directory (run: mkdir build && cd build && cmake ..)"
# fi

# Check for Bazel
if [[ -f "MODULE.bazel" ]]; then
    check_ok "Bazel configuration exists"

    if command -v bazel >/dev/null 2>&1; then
        if [[ -d "bazel-bin" ]]; then
            check_ok "Bazel build artifacts exist"
        else
            echo "  No Bazel build (run: bazel build //...)"
        fi
    fi
else
    check_warn "MODULE.bazel not found"
fi

section_header "WSL-Specific Checks"

if grep -qi microsoft /proc/version 2>/dev/null; then
    # Check DISPLAY
    if [[ -n "${DISPLAY:-}" ]]; then
        check_ok "DISPLAY is set: $DISPLAY"
    else
        check_warn "DISPLAY not set (GUI/OpenGL may not work)"
        echo "    Will be set automatically on next shell start"
    fi

    # Check if X server is reachable
    if command -v xset >/dev/null 2>&1; then
        if xset q &>/dev/null; then
            check_ok "X server is reachable"
        else
            check_warn "X server not reachable (ensure X server is running on Windows)"
        fi
    else
        echo "  xset not available (install x11-xserver-utils to test X server)"
    fi
else
    echo "  Not running in WSL"
fi

section_header "Summary"

echo ""
echo "Setup script: ./tools/scripts/setup_local_dev.sh"
echo "Documentation: docs/WSL_SETUP.md"
echo ""

# Return status based on critical checks
if [[ ! -d ".venv" ]] || ! command -v uv >/dev/null 2>&1; then
    echo -e "${COLOR_RED}❌ Critical components missing. Run setup script.${COLOR_END}"
    exit 1
elif [[ ! -f "$HOME/.vscode_profile" ]] || ! grep -q "source ~/.vscode_profile" "$HOME/.bashrc" 2>/dev/null; then
    echo -e "${COLOR_YELLOW}⚠ Shell configuration incomplete. Consider running setup script.${COLOR_END}"
    exit 0
else
    echo -e "${COLOR_GREEN}✅ Development environment is properly configured!${COLOR_END}"
    exit 0
fi
