# Modular Setup Scripts

This document describes the modular setup scripts that are shared between Docker and local development environments.

## Overview

The setup process has been refactored to use shared, modular scripts that ensure consistency between Docker containers and local development environments.

## Shared Components

### Configuration

- **`tool-versions.sh`** - Centralized version configuration for all development tools

### Installation Scripts

- **`install-system-packages.sh`** - Installs system packages using the same package lists as Docker
- **`install-uv.sh`** - Installs UV (Python package manager) with options for system/user installation
- **`install-bazel.sh`** - Installs Bazel/Bazelisk with configurable installation directory
- **`install-clangd.sh`** - Installs clangd language server and indexing tools

### Environment Setup

- **`setup-python-env.sh`** - Creates Python virtual environment and installs dependencies
- **`setup-precommit.sh`** - Configures pre-commit hooks

### Main Scripts

- **`setup_local_dev.sh`** - Main orchestration script for local development setup

## Usage

### Full Setup

```bash
./tools/scripts/setup_local_dev.sh
```

### Individual Components

Each script can be run independently:

```bash
# Install system packages
./tools/scripts/install-system-packages.sh

# Install UV
./tools/scripts/install-uv.sh

# Install Bazel
./tools/scripts/install-bazel.sh

# Install clangd
./tools/scripts/install-clangd.sh

# Setup Python environment
./tools/scripts/setup-python-env.sh

# Setup pre-commit hooks
./tools/scripts/setup-precommit.sh
```

## Benefits

1. **Consistency**: Same installation logic for Docker and local environments
2. **Maintainability**: Single source of truth for tool versions and installation procedures
3. **Modularity**: Individual components can be updated or run independently
4. **Reusability**: Scripts can be used by other projects or automation

## Docker Integration

The Docker build process now uses these shared scripts:

- Tool versions are centralized in `tool-versions.sh`
- UV, Bazel, and clangd installations use the shared scripts
- System package installation reuses the existing `install_apt_packages_from_list` script

This ensures that the Docker environment and local development environment use exactly the same tool versions and installation procedures.
