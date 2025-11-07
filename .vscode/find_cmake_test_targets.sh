#!/usr/bin/env bash

# Parse CMake test targets directly from CMakeLists.txt files
# This extracts targets created with add_test or le_add_test

WORKSPACE_DIR="${1:-/workspace/LineExtraction}"

# Function to extract test target name from add_test lines
extract_test_name() {
    local line="$1"
    # Remove add_test( or le_add_test(
    # Handles: add_test(NAME test_name ...) or add_test(test_name ...)
    if [[ "$line" =~ NAME[[:space:]]+([^[:space:]]+) ]]; then
        echo "${BASH_REMATCH[1]}"
    else
        # Extract first argument after add_test(
        local target=$(echo "$line" | sed -E 's/^[[:space:]]*(le_)?add_test\([[:space:]]*([^[:space:]()]+).*/\2/')
        # Skip if it's just "NAME" or empty
        if [[ "$target" != "NAME" ]] && [[ "$target" != "" ]]; then
            echo "$target"
        fi
    fi
}

# Find all CMakeLists.txt files and extract test targets
{
    # Method 1: Find add_test calls
    find "$WORKSPACE_DIR" -name "CMakeLists.txt" -type f | while read -r cmake_file; do
        grep -E "^[[:space:]]*(le_)?add_test[[:space:]]*\([[:space:]]*[^)]" "$cmake_file" | while read -r line; do
            # Skip commented lines
            if [[ "$line" =~ ^[[:space:]]*# ]]; then
                continue
            fi

            # Skip lines that are just "add_test(" without content
            if [[ "$line" =~ ^[[:space:]]*(le_)?add_test[[:space:]]*\([[:space:]]*$ ]]; then
                continue
            fi

            # Extract test name
            test_name=$(extract_test_name "$line")

            # Filter out variables, invalid names, and patterns
            if [[ "$test_name" =~ ^\$\{ ]] || \
               [[ "$test_name" == *" "* ]] || \
               [[ "$test_name" == "" ]] || \
               [[ "$test_name" == "NAME" ]] || \
               [[ "$test_name" == *'${'* ]] || \
               [[ "$test_name" == *'-${'* ]]; then
                continue
            fi

            echo "$test_name"
        done
    done

    # Method 2: Find auto-generated tests from le_add_library with AUTO_TESTS
    # These are test_*.cpp files in tests/ subdirectories
    find "$WORKSPACE_DIR" -type f -path "*/tests/test_*.cpp" | while read -r test_file; do
        # Extract test name from filename (without extension)
        test_name=$(basename "$test_file" .cpp)

        # Only include if not empty
        if [[ "$test_name" != "" ]]; then
            echo "$test_name"
        fi
    done
} | sort -u | grep -v "^$"
