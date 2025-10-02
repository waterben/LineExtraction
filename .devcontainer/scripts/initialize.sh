#!/usr/bin/env bash
set -euo pipefail
shopt -s nullglob globstar

writeWarning() {
    echo -e "\e[0;33mWARNING: $1"
    echo -e "\e[m"
}

# This script is started BEFORE the VScode dev container is launched
# in order to ensure the environment is properly set up on the host of execution.

# Ensure mounted path's are created before Docker launch to that they are not
# created by Docker daemon with root permissions
mkdir -p \
    "$HOME/.cache" \
    "$HOME/.ccache" \
    "$HOME/.ssh"

# Ensure file for local user exists. We will mount file into docker container via -v
touch $HOME/.netrc

# Add custom settings/environment variables
mkdir -p .devcontainer/scripts/custom
CUSTOM_CONFIG=.devcontainer/scripts/custom/initialize_custom.sh
if [ -f "$CUSTOM_CONFIG" ]; then
    echo "Custom initialization settings applied"
    source "$CUSTOM_CONFIG"
fi

bash docker/build_dev_images.sh "ubuntu:noble"

# SCRIPT_DIR will be the path to the .devcontainer folder independent of from where the script is being called from.
# --> The script can be called via VS Code DevContainer, e.g. bash ~/git_repos/xil-dev/.devcontainer/scripts/initialize.sh or simply from the .devcontainer folder: bash initialize.sh
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
env_file_path=$SCRIPT_DIR/../.env
if [ -f "$env_file_path" ]; then
   rm $env_file_path
fi

# USER: Needs to be set explicitly. Per default USER is available in Linux but not in Docker container. Used in VS Code tasks.
echo "http_proxy=${http_proxy:=""}
https_proxy=${https_proxy:=""}
all_proxy=${all_proxy:=""}
no_proxy=${no_proxy:=""}
HTTP_PROXY=${HTTP_PROXY:=""}
HTTPS_PROXY=${HTTPS_PROXY:=""}
ALL_PROXY=${ALL_PROXY:=""}
NO_PROXY=${NO_PROXY:=""}
SHELL=$SHELL
USER=$USER
CMAKE_CXX_COMPILER_LAUNCHER=ccache" \
> $env_file_path

# Check ptrace_scope setting if it exists
if [ -f "/proc/sys/kernel/yama/ptrace_scope" ]; then
    ptrace_value=$(sysctl --values kernel.yama.ptrace_scope 2>/dev/null || echo "")
    if [ -n "$ptrace_value" ] && [ "$ptrace_value" -eq 1 ] 2>/dev/null; then
        writeWarning "kernel.yama.ptrace_scope is set to 1 on your host system. You may face issues with debugging tools like gdb or valgrind. For example:\n\"Error getting authority: Error initializing authority: Could not connect: No such file or directory.\" You may want to set it to 0 by running:\n\"sudo nano /etc/sysctl.d/10-ptrace.conf\" and changing it there."
    fi
fi
