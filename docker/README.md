# Docker and DevContainer Setup

This directory contains Docker configurations and scripts for setting up consistent development environments. Docker provides isolated, reproducible environments that work identically across different machines and operating systems.

## Development Environment Options

### Docker/DevContainer (This Guide)

- **Best for:** Linux/macOS users, CI/CD pipelines
- **Pros:** Complete isolation, reproducible, works everywhere
- **Cons:** No OpenGL/GPU support, container overhead
- **Setup:** See below

### WSL (Windows Users)

- **Best for:** Windows users who need OpenGL/GUI support
- **Pros:** Native performance, OpenGL support, easier X server integration
- **Cons:** Windows-only, less isolation than Docker
- **Setup:** See [`../docs/WSL_SETUP.md`](../docs/WSL_SETUP.md)

### Native Linux

- **Best for:** Linux desktop users
- **Pros:** Native performance, full hardware access
- **Cons:** May affect system-wide packages
- **Setup:** Run `sudo ./tools/scripts/setup_local_dev.sh`

---

## Docker Development (Recommended for Linux/CI)

### VS Code DevContainer

1. Open project in VS Code
2. Install "Dev Containers" extension
3. Reopen in container when prompted

**Python versions available:**

- Python 3.11 (default)
- Python 3.8/3.10 (via virtual environments)

```bash
# Switch Python versions
source /opt/venv/deps/python3.8/bin/activate
deactivate  # Return to default
```

### Manual Docker Build

```bash
./build_dev_images.sh <ubuntu_version>  # focal, jammy, noble
```

### Docker Stages

The Dockerfile uses a multi-stage build:

- **`base`** - Core system packages and development tools
- **`devenv`** - Additional development environment tools and customizations

Build specific stages:

```bash
# Build only base stage
docker build --target base -t lineextraction:base .

# Build full development environment
docker build --target devenv -t lineextraction:devenv .
```

### Customization

Modify files in `devenv/custom/`:

- `entrypoint.sh` - Custom entrypoint
- `packages.txt` - Additional packages
- `requirements-pip.txt` - Python packages

Rebuild after changes: `docker rmi <image_name>`

## Local Development Setup

### Automated Setup (Recommended)

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

**Options:**

- `--help` - Show all options
- `--remove-tools` - Remove development tools
- `--remove-packages` - Remove APT packages

**Installs:** System packages, development tools (uv, bazel, clangd, gh), Python environment, pre-commit hooks.

### Manual Setup (Ubuntu/Debian)

1. **Install packages:**

   ```bash
   sudo apt install cmake build-essential vim gdb nano zsh
   ```

2. **Install development tools:**

   ```bash
   curl -fsSL https://raw.githubusercontent.com/waterben/LineExtraction/main/docker/scripts/install_base_tools | sudo bash
   curl -fsSL https://raw.githubusercontent.com/waterben/LineExtraction/main/docker/scripts/install_devenv_tools | sudo bash
   ```

3. **Setup Python environment:**

   ```bash
   uv venv .venv && source .venv/bin/activate && uv sync --locked
   pre-commit install
   ```

**Next:** See main `README.md` for build instructions.

## Docker Structure

### Dockerfile Architecture

Multi-stage build optimized for development:

1. **Base Stage:** Ubuntu + system packages + core tools
2. **DevEnv Stage:** Additional development tools + customizations

**Benefits:**

- Cached layers for faster rebuilds
- Flexible target selection
- Consistent tool versions across environments

## Scripts Reference

### Installation Scripts

- **`scripts/install_base_tools`** - Core tools: Bazelisk, UV, GitHub CLI, ruff
- **`scripts/install_devenv_tools`** - IDE tools: clangd, git-prompt
- **`scripts/install_apt_packages_from_list`** - APT package installer

### Package Configuration

- **`base/common_packages.txt`** - Base system packages
- **`devenv/common_packages.txt`** - Development environment packages

### Version Customization

Set environment variables before running scripts:

```bash
export BAZELISK_VERSION="1.25.0"
export CLANGD_VERSION="20.0.0"
sudo -E ./scripts/install_base_tools
```
