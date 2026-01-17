# WSL Development Setup

This guide describes how to set up the LineExtraction development environment in WSL (Windows Subsystem for Linux).

## Prerequisites

1. **WSL 2** with Ubuntu 24.04 or compatible
2. **VS Code** with the WSL extension installed
3. **X Server** for GUI applications (e.g., VcXsrv, X410)

## Quick Start

### 1. Run the Setup Script

From the project root, run:

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

This script will:

- Install all required system packages
- Install development tools (uv, bazel, clangd, etc.)
- Create a Python virtual environment in `.venv`
- Set up `.vscode_profile` for enhanced shell experience (git prompt, colors)
- Configure bash history persistence
- Install git pre-commit hooks
- Install VS Code extensions from `.devcontainer/devcontainer.json`
- Configure `.project_env` for automatic venv activation (shared with Docker)

The setup uses `.project_env` (in the project root) for project-specific environment setup.
This file is shared between Docker and WSL, ensuring consistent behavior in both environments.

### 2. Reload Your Shell

After the setup completes, reload your shell or source the profile:

```bash
source ~/.bashrc
```

You should now see:

- A colorized git-aware prompt showing branch and status
- Automatic activation of the Python virtual environment
- Page-up/page-down for command history search

### 3. Verify Setup

Run the status check script to verify everything is configured correctly:

```bash
./tools/scripts/check_dev_status.sh
```

This will check:

- System information and WSL detection
- Installed development tools
- Python environment and packages
- Shell configuration
- Git hooks
- VS Code settings
- Build system status
- WSL-specific configuration (DISPLAY, X server)

### 4. Open in VS Code

From WSL, open the project:

```bash
code .
```

VS Code will automatically:

- Use the local `.venv/bin/python` interpreter
- Enable clangd for C++ IntelliSense
- Configure all formatters and linters

## Features

### Shell Environment

The setup creates `~/.vscode_profile` with:

- **Git-aware prompt**: Shows current branch, dirty state, and upstream status
- **Colorized output**: Different colors for user, path, and git info
- **Persistent history**: Command history saved to `~/.cmd_history/.bash_history`
- **Auto venv activation**: Python virtual environment activates automatically

Example prompt:

```
user@host:~/workspace/LineExtraction (feature/improved-wsl-support)
>
```

### Python Environment

- Virtual environment in `.venv` (project-local)
- Automatically activated when shell starts
- All dependencies from `pyproject.toml` installed
- Pre-commit hooks configured

To manually activate:

```bash
source .venv/bin/activate
```

### VS Code Integration

The local setup configures:

- Python interpreter: `${workspaceFolder}/.venv/bin/python`
- clangd for C++ (installed to `/usr/local/bin/clangd`)
- All formatters and linters from `.pre-commit-config.yaml`

## Building the Project

### CMake Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)
```

### Bazel Build

```bash
bazel build //...
```

## Running GUI Applications

For OpenGL applications to work in WSL, you need an X server running on Windows:

1. **Install VcXsrv** (or X410)
2. **Start XLaunch** with "Disable access control" checked
3. **Set DISPLAY** (already configured in `.vscode_profile`):

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

1. Run your application:

```bash
./build/bin/app_line_analyzer
```

## Uninstalling

### Remove Development Tools Only

```bash
sudo ./tools/scripts/setup_local_dev.sh --remove-tools
```

This removes:

- Python virtual environment (`.venv`)
- Development tools (uv, bazel, clangd, etc.)
- Shell profile configurations
- Pre-commit hooks

### Remove System Packages

```bash
sudo ./tools/scripts/setup_local_dev.sh --remove-packages
```

This removes APT packages (but NOT system packages for stability).

### Remove Everything

```bash
sudo ./tools/scripts/setup_local_dev.sh --remove-tools
sudo ./tools/scripts/setup_local_dev.sh --remove-packages
```

## Comparison: WSL vs Docker

| Feature | WSL Setup | Docker Setup |
|---------|-----------|--------------|
| OpenGL Support | ✅ Yes (with X server) | ❌ No |
| Setup Complexity | Simple (one script) | Moderate (image build) |
| Python Version | 3.8+ (from pyproject.toml) | Multiple (3.8, 3.10, 3.11) |
| Python Location | `.venv` (project-local) | `/opt/venv/deps/python` (system) |
| Auto-activation | Yes (in shell profile) | Yes (in entrypoint) |
| Git Hooks | Installed by setup | Installed by entrypoint |
| Resource Usage | Native WSL | Container overhead |
| Isolation | WSL instance | Full container |

## Troubleshooting

### Check Setup Status

First, run the status check script to identify issues:

```bash
./tools/scripts/check_dev_status.sh
```

This provides a comprehensive overview of your development environment.

### Python not found in VS Code

1. Reload VS Code window: `Ctrl+Shift+P` → "Developer: Reload Window"
2. Check interpreter path: `Ctrl+Shift+P` → "Python: Select Interpreter"
3. Should show: `Python 3.x.x ('.venv': venv)`

### Git hooks not working

```bash
source .venv/bin/activate
pre-commit install --config .pre-commit-config.yaml
```

### Clangd not working

Check clangd is installed:

```bash
which clangd
clangd --version
```

If not found, re-run:

```bash
sudo ./tools/scripts/setup_local_dev.sh
```

### Shell profile not loading

Ensure `.bashrc` contains:

```bash
source ~/.vscode_profile
```

If not, manually add it or re-run the setup script.

## Advanced Configuration

### Custom Python Version

Edit `pyproject.toml` to change the required Python version:

```toml
[project]
requires-python = ">=3.11"
```

Then recreate the venv:

```bash
rm -rf .venv
source ~/.bashrc  # This will recreate .venv with correct version
```

### Multiple Workspaces

Each WSL instance can have its own setup. You can run:

```bash
wsl --install -d Ubuntu-24.04 LineExtraction
```

Then set up the environment independently in each instance.

## Tips

1. **Use WSL 2**: WSL 1 has limited functionality
2. **Windows Terminal**: Better experience than cmd.exe
3. **Git credentials**: Use Git Credential Manager for Windows
4. **File performance**: Keep source files in WSL filesystem (`/home/...`), not Windows (`/mnt/c/...`)

## Further Reading

- [VS Code WSL Documentation](https://code.visualstudio.com/docs/remote/wsl)
- [WSL 2 Best Practices](https://learn.microsoft.com/en-us/windows/wsl/setup/environment)
- [X Server Setup for WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
