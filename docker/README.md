# Docker and DevContainer Setup

This directory contains Docker configurations and scripts1. **Install system packages:**

   ```bash
   sudo apt update
   sudo apt install sudo ca-certificates curl git
   ```

2. **Install development packages:**

   ```bash
   sudo apt install cmake doxygen fontconfig moreutils jq build-essential llvm clang gcc g++ \
     ccache lcov locales zstd libarchive-tools shellcheck acl xxd qtbase5-dev
   ```

3. **Install DevEnv packages:**

   ```bash
   sudo apt install vim less gdb nano libarchive-tools zsh
   ```

4. **Install base development tools:**

   ```bash
   # Download and run the base tools installation script
   curl -fsSL https://raw.githubusercontent.com/waterben/LineExtraction/main/docker/scripts/install_base_tools -o /tmp/install_base_tools
   chmod +x /tmp/install_base_tools
   sudo /tmp/install_base_tools
   rm /tmp/install_base_tools
   ```

5. **Install development environment tools:**

   ```bash
   # Download and run the development environment tools installation script
   curl -fsSL https://raw.githubusercontent.com/waterben/LineExtraction/main/docker/scripts/install_devenv_tools -o /tmp/install_devenv_tools
   chmod +x /tmp/install_devenv_tools
   sudo /tmp/install_devenv_tools
   rm /tmp/install_devenv_tools
   ```

6. **Setup Python environment:**

   ```bash
   # Install Python dependencies
   uv venv .venv
   source .venv/bin/activate
   uv sync --locked
   ```

7. **Install pre-commit hooks:**

   ```bash
   pre-commit install --config .pre-commit-config.yaml
   ```g up consistent development environments. Docker provides isolated, reproducible environments that work identically across different machines and operating systems.

## Docker-based Development (Recommended)

### build_dev_images.sh

Builds Docker images for local development usage. This script must be executed from within the `docker` folder.

**Usage:**

```bash
./build_dev_images.sh <ubuntu_version>
```

**Supported Ubuntu versions:** `focal`, `jammy`, `noble`

### Dockerfile

A multi-stage Dockerfile that creates both base and development environment images. The base images are designed for use in both local development and CI environments.

**Key Features:**

- Uses the `install_base_tools` script to install development tools (Bazelisk, Buildifier, etc.)
- Tool-specific configurations (like `BAZELISK_BASE_URL`) are managed within installation scripts for better modularity
- Supports multiple Ubuntu versions through build arguments

### Development Environment Customization

Users can customize their development environment through several methods:

- **Custom entrypoint:** Modify `devenv/custom/entrypoint.sh`
- **Additional packages:** Add Debian packages to `devenv/custom/packages.txt`
- **Python packages:** Add pip packages to `devenv/custom/requirements-pip.txt`

**Important:** After modifying these files, trigger a Docker rebuild by deleting the old image:

```bash
docker rmi <image_name>
```

## Local Development Setup (Alternative to Docker)

For developers who prefer working without Docker, this section provides instructions to replicate the Docker environment locally. You'll install the same packages and tools used in the Docker container, providing maximum flexibility and native performance while maintaining consistency with the containerized environment.

### Automated Setup (Recommended)

Use the automated setup script to install all dependencies with a single command:

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

**Available options:**
- `sudo ./tools/scripts/setup_local_dev.sh` - Install complete development environment
- `sudo ./tools/scripts/setup_local_dev.sh --remove-tools` - Remove only development tools
- `sudo ./tools/scripts/setup_local_dev.sh --remove-packages` - Remove only APT packages
- `sudo ./tools/scripts/setup_local_dev.sh --help` - Show all options

This script installs:
- **System packages** from `docker/base/system_packages.txt` (sudo, ca-certificates, curl, git)
- **Development packages** from `docker/base/common_packages.txt` (cmake, build-essential, etc.)
- **DevEnv packages** from `docker/devenv/common_packages.txt` (vim, gdb, etc.)
- **Development tools** (uv, bazel, clangd, GitHub CLI, etc.)
- **Python environment** with `.venv` and dependencies
- **Pre-commit hooks**

### Manual Setup (Ubuntu/Debian)

If you prefer manual installation, follow these steps:

2. **Install additional development packages:**

   ```bash
   sudo apt install vim less gdb nano zsh
   ```

3. **Install development tools (Bazel, Buildifier, UV, etc.):**

   ```bash
   # Download and run the base tools installation script
   curl -fsSL https://raw.githubusercontent.com/waterben/LineExtraction/main/docker/scripts/install_base_tools -o /tmp/install_base_tools
   chmod +x /tmp/install_base_tools
   sudo /tmp/install_base_tools
   rm /tmp/install_base_tools
   ```

   **Installed tools:** Bazelisk (as `bazel`), Buildifier, GitHub CLI, UV, Ruff, and other essential development tools. The script automatically configures `BAZELISK_BASE_URL` for Bazel downloads.

4. **Install development environment tools:**

   ```bash
   # Download and run the development environment tools installation script
   curl -fsSL https://raw.githubusercontent.com/waterben/LineExtraction/main/docker/scripts/install_devenv_tools -o /tmp/install_devenv_tools
   chmod +x /tmp/install_devenv_tools
   sudo /tmp/install_devenv_tools
   rm /tmp/install_devenv_tools
   ```

   **Installed tools:** clangd, clangd indexing tools, and git-prompt for enhanced IDE support and improved shell experience.

5. **Configure environment variables (optional):**

   ```bash
   # If you need to customize Bazelisk behavior
   export BAZELISK_BASE_URL="https://github.com/bazelbuild/bazel/releases/download"
   ```

6. **Setup Python environment:**

   ```bash
   # Install Python dependencies
   uv venv .venv
   source .venv/bin/activate
   uv sync --locked
   ```

7. **Install pre-commit hooks:**

   ```bash
   pre-commit install --config .pre-commit-config.yaml
   ```

**Next Steps:** After completing the local setup, refer to the main `README.md` for build instructions and project usage.

## Scripts and Configuration Files

### Installation Scripts

These scripts are used by the Dockerfile and can also be used for local environment setup to maintain consistency:

- **`scripts/install_base_tools`** - Core development tools installation
  - **Tools:** Bazelisk, Buildifier, UV, GitHub CLI, actionlint, yq, ruff
  - **Docker Integration:** Used in base image layer
  - **Configuration:** Sets up `BAZELISK_BASE_URL` environment variable

- **`scripts/install_devenv_tools`** - Development environment enhancement tools
  - **Tools:** clangd, clangd indexing tools, git-prompt
  - **Docker Integration:** Used in development environment layer
  - **Purpose:** Enhanced IDE support and improved shell experience

- **`scripts/install_apt_packages_from_list`** - APT package management utility
  - **Purpose:** Automated package installation from text files
  - **Docker Integration:** Used throughout Dockerfile for package installation

### Docker Package Configuration

- **`base/common_packages.txt`** - Base system packages installed in Docker base layer
- **`devenv/common_packages.txt`** - Additional packages for development environment layer

These files define the package dependencies for different Docker image layers, ensuring consistent environments across builds.

### Tool Version Management

All tool versions are centrally managed within the installation scripts and can be customized by setting environment variables before running the scripts.

**Base Development Tools** (managed by `install_base_tools`):

| Tool | Environment Variable | Default Version |
|------|---------------------|----------------|
| Bazelisk | `BAZELISK_VERSION` | 1.26.0 |
| Buildifier | `BUILDIFIER_VERSION` | 8.2.1 |
| UV (Python Package Manager) | `UV_VERSION` | 0.8.4 |
| GitHub CLI | `GH_VERSION` | 2.76.2 |
| Actionlint | `ACTIONLINT_VERSION` | 1.7.7 |
| yq | `YQ_VERSION` | 4.47.1 |
| Ruff | `RUFF_VERSION` | 0.12.7 |

**Development Environment Tools** (managed by `install_devenv_tools`):

| Tool | Environment Variable | Default Version |
|------|---------------------|----------------|
| clangd | `CLANGD_VERSION` | 21.1.0 |

#### Custom Version Installation Example

```bash
# Install with custom tool versions
export BAZELISK_VERSION="1.25.0"
export CLANGD_VERSION="20.0.0"
sudo -E /path/to/install_base_tools
sudo -E /path/to/install_devenv_tools
```
