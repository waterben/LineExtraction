# Docker Development Environment

This guide covers using Docker and VS Code DevContainers for LineExtraction development. Docker provides isolated, reproducible environments that work identically across different machines.

## Quick Start

### VS Code DevContainer (Recommended)

1. **Install prerequisites:**
   - VS Code
   - Docker Desktop (Windows/Mac) or Docker Engine (Linux)
   - "Dev Containers" VS Code extension

2. **Open in container:**
   - Open project in VS Code
   - Click "Reopen in Container" when prompted
   - Or: `Ctrl+Shift+P` → "Dev Containers: Reopen in Container"

3. **Start developing:**
   - All tools pre-installed (CMake, Bazel, clangd, Python)
   - Extensions auto-configured
   - Python virtual environment ready

### Manual Docker Build

```bash
cd docker
./build_dev_images.sh <ubuntu_version>
```

**Available Ubuntu versions:**

- `focal` - Ubuntu 20.04 LTS
- `jammy` - Ubuntu 22.04 LTS
- `noble` - Ubuntu 24.04 LTS (default)

## Development Environment Comparison

| Feature | Docker/DevContainer | WSL | Native Linux |
|---------|---------------------|-----|--------------|
| Best for | Linux/macOS users, CI/CD | Windows users needing GUI | Linux desktop users |
| OpenGL Support | ❌ No | ✅ Yes (WSLg/X server) | ✅ Yes |
| Setup Complexity | Simple (automated) | Simple (one script) | Simple (one script) |
| Isolation | Complete | WSL instance | None |
| Performance | Container overhead | Native | Native |
| GPU Support | ❌ No | ⚠️ Limited | ✅ Yes |
| Reproducibility | ✅ Excellent | ⚠️ Good | ⚠️ Good |

**Recommendations:**

- **Windows users needing GUI/OpenGL:** Use [WSL](WSL.md)
- **Linux/macOS users:** Use Docker DevContainer (this guide)
- **CI/CD pipelines:** Use Docker DevContainer
- **Linux desktop with GUI needs:** Use [native setup](../docker/README.md#local-development-setup)

## Python Environment

The DevContainer provides multiple Python versions:

- **Python 3.11** (default in container)
- **Python 3.8** (virtual environment: `/opt/venv/deps/python3.8`)
- **Python 3.10** (virtual environment: `/opt/venv/deps/python3.10`)

### Switching Python Versions

```bash
# Switch to Python 3.8
source /opt/venv/deps/python3.8/bin/activate

# Switch to Python 3.10
source /opt/venv/deps/python3.10/bin/activate

# Return to default (3.11)
deactivate
```

VS Code automatically uses the correct Python interpreter based on the active virtual environment.

## Building the Project

### CMake Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)

# Run tests
ctest
```

See [`CMAKE.md`](CMAKE.md) for complete CMake documentation.

### Bazel Build

```bash
# Detect features (Qt5, OpenGL, CUDA)
./tools/scripts/detect_bazel_features.sh

# Build all
bazel build //...

# Run tests
bazel test //...
```

See [`BAZEL.md`](BAZEL.md) for complete Bazel documentation.

## Customization

### Custom Packages

Add system packages to `docker/devenv/custom/packages.txt`:

```text
htop
tree
neovim
```

### Custom Python Packages

Add Python packages to `docker/devenv/custom/requirements-pip.txt`:

```text
numpy==1.24.0
matplotlib
```

### Custom Entrypoint

Create `docker/devenv/custom/entrypoint.sh`:

```bash
#!/bin/bash
echo "Custom initialization"
# Your custom setup here
```

After changes, rebuild the container:

```bash
docker rmi lineextraction:devenv
# Reopen in VS Code - will rebuild automatically
```

## Docker Architecture

### Multi-Stage Build

The Dockerfile uses a multi-stage build for efficiency:

```
┌─────────────────────────────────────────┐
│ Stage 1: base                           │
│ - Ubuntu base image                     │
│ - System packages (build-essential, etc)│
│ - Core development tools                │
│ - Python 3.8, 3.10, 3.11                │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│ Stage 2: devenv                         │
│ - Additional dev tools (clangd, gh)     │
│ - Git prompt configuration              │
│ - Pre-commit hooks setup                │
│ - Custom packages (if any)              │
└─────────────────────────────────────────┘
```

### Build Specific Stages

```bash
# Build only base stage
docker build --target base -t lineextraction:base -f docker/Dockerfile .

# Build full development environment
docker build --target devenv -t lineextraction:devenv -f docker/Dockerfile .
```

## Scripts Overview

### Installation Scripts (`docker/scripts/`)

| Script | Purpose |
|--------|---------|
| `install_base_tools` | Core tools: Bazelisk, UV, GitHub CLI, ruff |
| `install_devenv_tools` | IDE tools: clangd, git-prompt |
| `install_apt_packages_from_list` | Installs packages from text files |

### Package Lists

| File | Description |
|------|-------------|
| `docker/base/common_packages.txt` | Base system packages |
| `docker/devenv/common_packages.txt` | Development environment packages |
| `docker/devenv/custom/packages.txt` | User custom packages |

### Entrypoint

The container entrypoint (`docker/devenv/entrypoint.sh`) automatically:

1. Sources `.project_env` (git prompt, colors, venv)
2. Runs custom entrypoint if exists
3. Installs pre-commit hooks if needed
4. Drops to user shell

## Tool Versions

Default versions (can be customized via environment variables):

| Tool | Version | Environment Variable |
|------|---------|---------------------|
| Bazelisk | 1.27.0 | `BAZELISK_VERSION` |
| clangd | 20.0.0 | `CLANGD_VERSION` |
| UV | latest | `UV_VERSION` |
| GitHub CLI | latest | - |
| ruff | latest | - |

### Custom Tool Versions

```bash
# Set before running scripts
export BAZELISK_VERSION="1.27.0"
export CLANGD_VERSION="19.0.0"

sudo -E ./docker/scripts/install_base_tools
```

## VS Code Integration

The DevContainer automatically configures VS Code with:

### Extensions (from `.devcontainer/devcontainer.json`)

- **C/C++:** clangd, CMake Tools
- **Python:** Python, Pylance
- **Bazel:** Bazel
- **Git:** GitLens
- **Formatters:** Prettier, Ruff
- **Linters:** Actionlint, YAML

### Settings

- Python interpreter: `/opt/venv/deps/python3.11/bin/python`
- clangd for C++ IntelliSense
- Auto-formatting on save
- Pre-commit hooks enabled

## Container Management

### Rebuild Container

```bash
# From VS Code
Ctrl+Shift+P → "Dev Containers: Rebuild Container"

# Or manually
docker rmi lineextraction:devenv
# Reopen in VS Code
```

### Clean Everything

```bash
# Remove all containers and images
docker system prune -a

# Remove specific image
docker rmi lineextraction:devenv
```

### View Container Logs

```bash
# Running containers
docker ps

# Container logs
docker logs <container_id>
```

## Troubleshooting

### Container fails to start

```bash
# Check Docker is running
docker ps

# View build logs
docker build --progress=plain -t lineextraction:devenv -f docker/Dockerfile .

# Check disk space
docker system df
```

### Extensions not loading

```bash
# Rebuild container
Ctrl+Shift+P → "Dev Containers: Rebuild Container"

# Check extension install logs
View → Output → Dev Containers
```

### Python environment issues

```bash
# Check active Python
which python
python --version

# List installed packages
pip list

# Reinstall dependencies
cd /workspace
uv sync --locked
```

### Clangd not working

```bash
# Check clangd installation
which clangd
clangd --version

# Generate compile_commands.json (for CMake)
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..

# Or for Bazel
./tools/scripts/generate_compile_commands.sh
```

### Performance issues

```bash
# Limit container resources in Docker Desktop
Settings → Resources → Advanced

# Or use .wslconfig (WSL backend)
# C:\Users\<username>\.wslconfig
[wsl2]
memory=8GB
processors=4
```

## Advanced Usage

### Attach to Running Container

```bash
# List running containers
docker ps

# Attach to container
docker exec -it <container_id> bash
```

### Run Commands in Container

```bash
# From host, run command in container
docker exec <container_id> bazel build //...

# Or via VS Code terminal (already in container)
bazel build //...
```

### Volume Mounts

The DevContainer mounts:

- **Project root** → `/workspace` (read-write)
- **Docker socket** → `/var/run/docker.sock` (for Docker-in-Docker)
- **Git credentials** → `~/.gitconfig` (if exists)

### Customize DevContainer

Edit `.devcontainer/devcontainer.json`:

```json
{
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-vscode.cpptools",
        "your-custom-extension"
      ],
      "settings": {
        "your.custom.setting": "value"
      }
    }
  }
}
```

## Troubleshooting

### Pre-commit Hooks Fail After Switching Between Docker and Local

**Symptom:** Git commits fail in VS Code with `pre-commit not found` or similar errors, but work fine in the terminal.

**Cause:** The pre-commit hook stores an absolute path to the Python interpreter. When switching between Docker (`/opt/venv/deps/python/bin/python3`) and local development (`~/.venv/bin/python3`), the stored path becomes invalid.

**Solution:** Reinstall pre-commit hooks in your current environment:

```bash
# In Docker container
pre-commit install --install-hooks

# Or locally (with venv activated)
source .venv/bin/activate
pre-commit install --install-hooks
```

**Verify the fix:**

```bash
head -10 .git/hooks/pre-commit
# Should show INSTALL_PYTHON pointing to your current environment
```

> **Note:** You need to run `pre-commit install` each time you switch between Docker and local development.

### Container Cannot Access GPU/OpenGL

Docker containers have no GPU/OpenGL support in this project. For GPU-accelerated work or GUI applications:

- **Windows:** Use [WSL with WSLg](WSL.md)
- **Linux:** Use native development with `./tools/scripts/setup_local_dev.sh`

### Bazel Cache Issues After Container Rebuild

```bash
# Clean Bazel cache
bazel clean --expunge

# Rebuild
bazel build //...
```

## Local Development (Without Docker)

For native Linux/WSL development without Docker, see:

- **WSL Setup:** [`WSL.md`](WSL.md)
- **Native Setup:** [`../docker/README.md`](../docker/README.md#local-development-setup)

Both use the same setup script:

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

## Further Reading

- [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [Docker Documentation](https://docs.docker.com/)
- [CMake Build System](CMAKE.md)
- [Bazel Build System](BAZEL.md)
- [WSL Setup (Windows)](WSL.md)
- [Docker README (Details)](../docker/README.md)
