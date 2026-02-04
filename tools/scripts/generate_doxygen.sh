#!/bin/bash
# ==============================================================================
# Doxygen Documentation Generator
# ==============================================================================
# Generates API documentation using Doxygen.
# Can be run directly or via: bazel run //:doxygen
#
# Output: docs/generated/html/index.html
# ==============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Find project root (handle both direct execution and bazel run)
if [[ -n "${BUILD_WORKSPACE_DIRECTORY}" ]]; then
    # Running via bazel run
    PROJECT_ROOT="${BUILD_WORKSPACE_DIRECTORY}"
elif [[ -f "Doxyfile.in" ]]; then
    PROJECT_ROOT="$(pwd)"
else
    # Find project root from script location
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi

cd "${PROJECT_ROOT}"

echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}           Generating Doxygen Documentation${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Check doxygen is installed
if ! command -v doxygen &>/dev/null; then
    echo -e "${RED}ERROR: doxygen is not installed.${NC}"
    echo "Install with: sudo apt install doxygen graphviz"
    exit 1
fi

# Create output directory
mkdir -p docs/generated

# Generate Doxyfile from template
echo "Generating Doxyfile..."
sed -e "s|@CMAKE_CURRENT_SOURCE_DIR@|${PROJECT_ROOT}|g" \
    Doxyfile.in > build/Doxyfile

# Run doxygen
echo "Running Doxygen..."
doxygen build/Doxyfile

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}Documentation generated successfully!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo "Open: ${PROJECT_ROOT}/docs/generated/html/index.html"
echo ""

# Try to show some stats
if [[ -f "docs/generated/html/annotated.html" ]]; then
    CLASS_COUNT=$(grep -c "class=\"el\"" docs/generated/html/annotated.html 2>/dev/null || echo "?")
    echo "Documented classes/structs: ${CLASS_COUNT}"
fi
