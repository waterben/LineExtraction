# WSL Development Setup

This guide describes how to set up the LineExtraction development environment in WSL (Windows Subsystem for Linux).

## Prerequisites

1. **WSL 2** with Ubuntu 24.04 or compatible
2. **VS Code** with the WSL extension installed
3. **X Server** for GUI applications (e.g., VcXsrv, X410, or WSLg on Windows 11+)

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
- Configure `.project_env` for enhanced shell experience (git prompt, colors, venv activation)
- Configure bash history persistence
- Install git pre-commit hooks
- Install VS Code extensions from `.devcontainer/devcontainer.json`
- Detect and configure Bazel features (Qt5, OpenGL, CUDA)

The setup uses `.project_env` (in the project root) for project-specific environment setup.
This file is shared between Docker and WSL, ensuring consistent behavior in both environments.

### 2. Reload Your Shell

After the setup completes, reload your shell or open a new terminal in VS Code.
The `.project_env` will be automatically sourced when you open a terminal.

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

The setup creates `.project_env` with:

- **Git-aware prompt**: Shows current branch, dirty state, and upstream status
- **Colorized output**: Different colors for user, path, and git info
- **Persistent history**: Command history saved to `~/.cmd_history/.bash_history`
- **Auto venv activation**: Python virtual environment activates automatically
- **DISPLAY configuration**: Automatic X server display setup (WSLg on Windows 11+)

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
- Terminal integration via `.project_env`

## Building the Project

### CMake Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)
```

### Bazel Build

```bash
# First run: detect features (Qt5, OpenGL, CUDA)
./tools/scripts/detect_bazel_features.sh

# Build all
bazel build //...

# Build specific targets
bazel build //apps/line_analyzer:app_line_analyzer
```

See [`BAZEL.md`](BAZEL.md) for complete documentation.

## Running GUI Applications

### Windows 11+ (WSLg)

Windows 11 includes WSLg (WSL GUI support) - no additional setup needed!
The setup script automatically configures `DISPLAY=:0`.

Just run your application:

```bash
./build/bin/app_line_analyzer
# or
bazel run //apps/line_analyzer:app_line_analyzer
```

### Windows 10 (X Server Required)

For Windows 10, you need an X server:

1. **Install VcXsrv** or X410
2. **Start XLaunch** with "Disable access control" checked
3. **DISPLAY** is automatically configured in `.project_env`

Then run your application:

```bash
./build/bin/app_line_analyzer
```

## Git Credential Setup

### Problem

When using WSL, `git push` may hang without asking for credentials because:

1. Git tries to use the Windows Git Credential Manager
2. The credential manager runs in the background and waits for GUI input
3. You don't see the authentication window

### Recommended Solution: SSH Keys

The most secure and reliable option:

1. **Generate SSH key in WSL:**

   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

2. **Add SSH key to GitHub:**

   ```bash
   cat ~/.ssh/id_ed25519.pub
   # Copy the output and add it to GitHub Settings → SSH Keys
   ```

3. **Change remote URL to SSH:**

   ```bash
   git remote set-url origin git@github.com:waterben/LineExtraction.git
   ```

4. **Test connection:**

   ```bash
   ssh -T git@github.com
   ```

5. **Now push works:**

   ```bash
   git push
   ```

### Alternative: Windows Git Credential Manager

Use the same credentials as Windows Git:

```bash
git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/bin/git-credential-manager.exe"
```

On first push, a Windows authentication window will appear.

### Alternative: Git Credential Manager Core

If you have Git Credential Manager Core:

```bash
git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/libexec/git-core/git-credential-manager.exe"
```

### Alternative: Store Credentials in WSL

Store credentials directly in WSL (less secure):

```bash
# Store credentials in plain text (use with caution)
git config --global credential.helper store

# Or use cache (credentials expire after timeout)
git config --global credential.helper cache
git config --global credential.cache.timeout 3600  # 1 hour
```

### Auto-Start SSH Agent

Add to your `~/.bashrc` (or the setup script does this automatically):

```bash
# Auto-start SSH agent
if [ -z "$SSH_AUTH_SOCK" ]; then
    eval "$(ssh-agent -s)" >/dev/null 2>&1
    ssh-add ~/.ssh/id_ed25519 2>/dev/null
fi
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
| OpenGL Support | ✅ Yes (WSLg or X server) | ❌ No |
| Setup Complexity | Simple (one script) | Moderate (image build) |
| Python Version | 3.8+ (from pyproject.toml) | Multiple (3.8, 3.10, 3.11) |
| Python Location | `.venv` (project-local) | `/opt/venv/deps/python` (system) |
| Auto-activation | Yes (via `.project_env`) | Yes (in entrypoint) |
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

### Git push hangs

See [Git Credential Setup](#git-credential-setup) above. Recommended: use SSH keys.

If still hangs:

1. Kill the process: `Ctrl+C`
2. Check running processes:

   ```bash
   ps aux | grep git
   ```

3. Kill hung processes:

   ```bash
   pkill -f git-credential
   ```

4. Try with verbose output:

   ```bash
   GIT_TRACE=1 GIT_CURL_VERBOSE=1 git push
   ```

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

VS Code terminal should automatically source `.project_env`. Check:

```bash
echo $LE_PROJECT_DIR
# Should show: /home/username/workspace/LineExtraction
```

If not set, check `.vscode/settings.json` contains:

```json
{
  "terminal.integrated.env.linux": {
    "LE_PROJECT_DIR": "${workspaceFolder}"
  }
}
```

### GUI applications don't work

**Windows 11 (WSLg):**

- Should work out of the box
- Check `echo $DISPLAY` shows `:0`
- Try: `xcalc` (simple test app)

**Windows 10 (X Server):**

- Ensure X server is running (VcXsrv/X410)
- Check `echo $DISPLAY` shows correct value
- Test X server: `xcalc`
- If `xcalc` works but app doesn't, it's an OpenGL issue

### Permission denied (SSH)

If using SSH and getting permission denied:

```bash
# Check SSH agent is running
eval "$(ssh-agent -s)"

# Add your key
ssh-add ~/.ssh/id_ed25519

# Verify key is loaded
ssh-add -l

# Test GitHub connection
ssh -T git@github.com
```

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
# Open new terminal - .project_env will recreate .venv
```

### Multiple Workspaces

Each WSL instance can have its own setup. You can create multiple distributions:

```bash
wsl --install -d Ubuntu-24.04
```

Then set up the environment independently in each instance.

### Custom X Server Display

If using a custom X server configuration, edit `.project_env`:

```bash
# For custom IP/port
export DISPLAY=192.168.1.100:0.0
```

## Performance Tips

1. **Use WSL 2**: WSL 1 has limited functionality and poor I/O performance
2. **Keep files in WSL**: Store project in `/home/...`, not `/mnt/c/...` (10-100x faster)
3. **Windows Terminal**: Better experience than cmd.exe or PowerShell
4. **Bazel disk cache**: Automatically configured to `~/.cache/bazel/`
5. **Parallel builds**: Use `-j$(nproc)` for CMake or `--jobs=auto` for Bazel

## Tips & Best Practices

1. **File location**: Always work in `/home/username/...`, never in `/mnt/c/...`
2. **Git credentials**: Use SSH keys for best experience
3. **VS Code**: Open from WSL with `code .` for proper integration
4. **Line endings**: Git should be configured for LF (done by setup script)
5. **Windows 11**: Use WSLg for automatic GUI support
6. **Windows 10**: Install VcXsrv or X410 for GUI apps

## Further Reading

- [VS Code WSL Documentation](https://code.visualstudio.com/docs/remote/wsl)
- [WSL 2 Best Practices](https://learn.microsoft.com/en-us/windows/wsl/setup/environment)
- [WSLg GUI Apps](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
- [Bazel Documentation](BAZEL.md)
- [Docker Setup](../docker/README.md)
