#!/usr/bin/env bash

# Parse CMake targets directly from CMakeLists.txt files
# This works without needing a configured build directory

WORKSPACE_DIR="${1:-/workspace/LineExtraction}"

# Function to extract target name from add_executable lines
extract_target_name() {
    local line="$1"
    # Remove add_executable( or le_add_executable(
    echo "$line" | sed -E 's/^[[:space:]]*(le_)?add_executable\([[:space:]]*([^[:space:]]+).*/\2/'
}

# Always include standard targets
echo "all"
echo "clean"
echo "install"

# Find all CMakeLists.txt files and extract executable targets only
find "$WORKSPACE_DIR" -name "CMakeLists.txt" -type f | while read -r cmake_file; do
    # Extract add_executable calls only (including le_add_executable variants)
    grep -E "^[[:space:]]*(le_)?add_executable[[:space:]]*\(" "$cmake_file" | while read -r line; do
        # Skip commented lines
        if [[ "$line" =~ ^[[:space:]]*# ]]; then
            continue
        fi

        # Extract target name
        target=$(extract_target_name "$line")

        # Filter out variables and invalid names
        if [[ "$target" =~ ^\$\{ ]] || [[ "$target" == *" "* ]] || [[ "$target" == "" ]]; then
            continue
        fi

        echo "$target"
    done
done | sort -u | grep -v "^$"
